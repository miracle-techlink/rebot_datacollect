#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""`starai_to_rebot_leader` — StarAI Violin leader → reBot B601-RS 关节空间 teleop。

内部持有一个 ``StaraiViolinLeader``(复用其标定),``get_action`` 把 leader 输出映射成
reBot 关节空间的绝对目标角,键与 ``rebot_follower`` 的 action_features 完全一致
(``shoulder_pan.pos`` ... ``wrist_roll.pos`` + ``gripper.pos``)。见 config 里的映射说明。
"""

import logging

from lerobot.teleoperators.teleoperator import Teleoperator

from .config_starai_to_rebot_leader import StaraiToRebotLeaderConfig

logger = logging.getLogger(__name__)

REBOT_ARM_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]


def _parse_flip(s: str) -> list[float]:
    sign = [1.0] * 6
    for tok in s.split(","):
        tok = tok.strip()
        if tok:
            sign[int(tok) - 1] = -1.0
    return sign


class StaraiToRebotLeader(Teleoperator):
    config_class = StaraiToRebotLeaderConfig
    name = "starai_to_rebot_leader"

    def __init__(self, config: StaraiToRebotLeaderConfig):
        super().__init__(config)
        self.config = config

        from lerobot.teleoperators.starai_violin_leader import (
            StaraiViolinLeader,
            StaraiViolinLeaderConfig,
        )

        self._leader = StaraiViolinLeader(
            StaraiViolinLeaderConfig(
                port=config.port,
                baudrate=config.baudrate,
                arm_servo_ids=config.arm_servo_ids,
                gripper_servo_id=config.gripper_servo_id,
                id=config.leader_id,
            )
        )
        self._sign = _parse_flip(config.flip)
        self._leader_home: list[float] | None = None
        self._cmd_arm: list[float] | None = None  # 启动 ramp 用:当前臂输出目标
        self._ramped_in: bool = False             # 启动 ramp 是否已收敛(收敛后直通)

        # 夹爪闭合端过冲:ratio=0 时目标压过闭合位,产生持续夹持力。
        close_dir = -1.0 if config.grip_close_deg <= config.grip_open_deg else 1.0
        self._grip_close_eff = config.grip_close_deg + close_dir * config.grip_clamp_deg

    # ---------------- features ----------------
    @property
    def action_features(self) -> dict[str, type]:
        ft = {f"{m}.pos": float for m in REBOT_ARM_MOTORS}
        ft["gripper.pos"] = float
        return ft

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    # ---------------- lifecycle(全部委托内部 leader) ----------------
    @property
    def is_connected(self) -> bool:
        return self._leader.is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._leader.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        self._leader.connect(calibrate=calibrate)
        self._leader_home = None   # 下一次 get_action 重新确定参考基准
        self._cmd_arm = None       # 启动 ramp 从 home 重新开始
        self._ramped_in = False

    def calibrate(self) -> None:
        self._leader.calibrate()

    def configure(self) -> None:
        pass

    def rearm_ramp(self) -> None:
        """重新武装启动限速 ramp:下一次 get_action 从**当前保持位**(``_cmd_arm``,即上一条结束时
        的目标)平滑滑到 leader 当前绝对位姿,而不是直通。闸门式采集里每条开录前调用 —— 因为"回车
        等待"期间机械臂冻结、主臂可能被挪动,不重新限速则下一条起步会无限速弹射。不重置 ``_cmd_arm``
        (从保持位 ramp,而非回 home);``_leader_home`` 绝对参考也保持不变。"""
        self._ramped_in = False

    # ---------------- action ----------------
    def get_action(self) -> dict[str, float]:
        la = self._leader.get_action()  # joint_1..6.pos(deg-from-home)+ gripper.pos([0,1])
        leader = [float(la[f"joint_{i + 1}.pos"]) for i in range(6)]
        if self._leader_home is None:
            # 参考基准:absolute → leader 标定零位(全 0),leader 绝对角恒定映射(启动即对上绝对位姿);
            # 非 absolute → 首帧 leader 读数(旧的进入即锚定,无起步跳变)。
            self._leader_home = [0.0] * 6 if self.config.absolute else list(leader)

        # 绝对目标(reBot 关节空间)
        target = [
            self.config.rebot_home_deg[i]
            + self._sign[i] * self.config.scale * (leader[i] - self._leader_home[i])
            for i in range(6)
        ]

        # 启动 ramp:从 home 坐姿限速滑向目标(纯输出插值,不夹传感器 → 不抖),收敛后直通稳态 1:1。
        step = self.config.startup_ramp_deg_per_step
        if self._cmd_arm is None:
            self._cmd_arm = list(self.config.rebot_home_deg)
        if not self._ramped_in and step and step > 0.0:
            residual = 0.0
            for i in range(6):
                d = max(-step, min(step, target[i] - self._cmd_arm[i]))
                self._cmd_arm[i] += d
                residual = max(residual, abs(target[i] - self._cmd_arm[i]))
            arm = list(self._cmd_arm)
            if residual < 0.5:
                self._ramped_in = True
        else:
            arm = target
            self._cmd_arm = list(target)

        out: dict[str, float] = {}
        for i, m in enumerate(REBOT_ARM_MOTORS):
            out[f"{m}.pos"] = arm[i]

        # 夹爪:leader ratio [grip_ratio_min, grip_ratio_max] → [close_eff, open]
        raw = float(la.get("gripper.pos", 0.0))
        denom = max(self.config.grip_ratio_max - self.config.grip_ratio_min, 1e-3)
        ratio = min(1.0, max(0.0, (raw - self.config.grip_ratio_min) / denom))
        out["gripper.pos"] = self._grip_close_eff + ratio * (self.config.grip_open_deg - self._grip_close_eff)
        return out

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        self._leader.disconnect()
