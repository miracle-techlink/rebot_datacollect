#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import json
import logging
import threading
import time
from pathlib import Path

import numpy as np
import serial

from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_starai_violin_leader import StaraiViolinLeaderConfig

logger = logging.getLogger(__name__)


class StaraiViolinLeader(Teleoperator):
    """StarAI Violin leader arm read over the Fashionstar UART servo bus.

    Calibration records BOTH a zero (homing) pose AND each joint's range of motion.
    ``get_action`` outputs a NORMALIZED position per joint, centered on the zero pose
    and scaled by that joint's half-range → ~[-1, 1] (keys ``joint_1.pos`` ...
    ``joint_6.pos`` + ``gripper.pos``). The follower maps this into its own
    zero+half-range. Raw angle readings are unwrapped during the range sweep so a
    servo crossing the ±180° boundary does not corrupt the recorded range.
    """

    config_class = StaraiViolinLeaderConfig
    name = "starai_violin_leader"

    def __init__(self, config: StaraiViolinLeaderConfig):
        # all set before super().__init__ (may auto-load calibration)
        self._home = None       # zero-pose raw angles (deg)
        self._range_min = None  # per-joint min (deg, unwrapped, home frame)
        self._range_max = None
        super().__init__(config)
        self.config = config
        self._uart: serial.Serial | None = None
        self._uservo = None
        self._servo_ids = [*config.arm_servo_ids, config.gripper_servo_id]
        self._keys = [f"joint_{i + 1}.pos" for i in range(len(config.arm_servo_ids))] + ["gripper.pos"]

    @property
    def action_features(self) -> dict[str, type]:
        return {k: float for k in self._keys}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._uart is not None and self._uart.is_open

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        from uservo import UartServoManager

        self._uart = serial.Serial(
            port=self.config.port, baudrate=self.config.baudrate,
            parity=serial.PARITY_NONE, stopbits=1, bytesize=8, timeout=0,
        )
        self._uservo = UartServoManager(self._uart, srv_num=self.config.gripper_servo_id + 1)
        found = set(self._uservo.servos.keys())
        missing = set(self._servo_ids) - found
        if missing:
            raise ConnectionError(
                f"{self} connected but servos {sorted(missing)} not found (found {sorted(found)}). "
                f"Check power/baudrate ({self.config.baudrate})."
            )
        if not self.is_calibrated and calibrate:
            logger.info(f"{self} not calibrated; running calibration.")
            self.calibrate()
        logger.info(f"{self} connected, servos: {sorted(found)}")

    @property
    def is_calibrated(self) -> bool:
        return self._home is not None and self._range_min is not None and self._range_max is not None

    def _read_raw_deg(self) -> np.ndarray:
        self._uservo.query_all_srv_angle()
        return np.array([float(self._uservo.servos[sid].angle) for sid in self._servo_ids])

    def calibrate(self) -> None:
        input(f"\n[{self}] ① 零位:把主臂移到【零位姿态】(与从臂零位对应),扶稳后按 Enter...")
        self._home = self._read_raw_deg()
        print(f"零位记录: {np.round(self._home, 1).tolist()}")

        input("② 限位:按 Enter 开始,然后把【每个关节都缓慢转到两端极限】来回扫一遍...")
        print("记录中... 所有关节都转满后按 Enter 结束。")
        cont = self._home.copy()          # continuous (unwrapped) angle, starts at home
        prev = self._read_raw_deg()
        mins = self._home.copy()
        maxs = self._home.copy()
        done = threading.Event()
        threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
        while not done.is_set():
            raw = self._read_raw_deg()
            cont = cont + (((raw - prev + 180.0) % 360.0) - 180.0)  # unwrap step
            prev = raw
            mins = np.minimum(mins, cont)
            maxs = np.maximum(maxs, cont)
            print("  min: " + " ".join(f"{m:+6.1f}" for m in mins)
                  + " | max: " + " ".join(f"{m:+6.1f}" for m in maxs), end="\r")
            time.sleep(0.02)
        self._range_min, self._range_max = mins, maxs
        self._save_calibration()
        print(f"\n标定完成,已保存到 {self.calibration_fpath}")
        logger.info(f"{self} home={np.round(self._home,1).tolist()} "
                    f"min={np.round(mins,1).tolist()} max={np.round(maxs,1).tolist()}")

    def configure(self) -> None:
        pass

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f:
            d = json.load(f)
        self._home = np.asarray(d["homing_offset_deg"], dtype=float)
        self._range_min = np.asarray(d["range_min_deg"], dtype=float)
        self._range_max = np.asarray(d["range_max_deg"], dtype=float)

    def _save_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath, "w") as f:
            json.dump({"homing_offset_deg": self._home.tolist(),
                       "range_min_deg": self._range_min.tolist(),
                       "range_max_deg": self._range_max.tolist()}, f, indent=4)

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        raw = self._read_raw_deg()
        # unwrap current reading into the home frame (handles ±180 wrap)
        rel = ((raw - self._home + 180.0) % 360.0) - 180.0  # deg from home, in [-180,180]
        # Arm joints: emit degrees-from-home (1:1 direct-angle mapping downstream).
        out = {k: float(rel[i]) for i, k in enumerate(self._keys[:-1])}
        # Gripper (last key): emit travel FRACTION [0,1] over its full range (its home
        # may sit at an end, so a centered [-1,1] would only use half the travel).
        gi = len(self._keys) - 1
        span_g = max(self._range_max[gi] - self._range_min[gi], 1e-3)
        frac = (raw[gi] - self._range_min[gi]) / span_g
        out[self._keys[gi]] = float(np.clip(frac, 0.0, 1.0))
        return out

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        if self._uart is not None and self._uart.is_open:
            self._uart.close()
        self._uart = None
        self._uservo = None
        logger.info(f"{self} disconnected.")
