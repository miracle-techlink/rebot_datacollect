#!/usr/bin/env bash
# 修复:某些 pyav 构建不支持 gray12le 的 numpy 转换(from_ndarray/to_ndarray 抛
# "Conversion ... with format `gray12le` is not yet supported"),导致带深度的数据集在
# save_episode(深度视频编码)时崩溃 / 或解码时崩溃。
#
# 本脚本幂等地打两处 lerobot 核心补丁,绕开该 pyav bug,保持 gray12le/hevc/mp4 设计不变:
#   1) datasets/depth_utils.py  编码:用 VideoFrame 构造器 + write_u16_plane 建帧(替 from_ndarray)
#   2) datasets/video_utils.py  解码:手动读 u16 plane(替 to_ndarray(gray12le))
# 已验证:hevc/gray12le/mp4 端到端 round-trip 正常,深度误差=纯 12-bit 量化(~1mm),无编码损失。
# lerobot 升级会覆盖核心文件 → 升级后重跑本脚本。
#
# 用法: LEROBOT_SRC=/path/to/lerobot bash install_depthfix.sh
set -e
LEROBOT_SRC="${LEROBOT_SRC:-$HOME/lingbot/lerobot}"
LB="$LEROBOT_SRC/src/lerobot"
[ -d "$LB" ] || { echo "!! 找不到 lerobot 源码: $LB (设 LEROBOT_SRC=lerobot仓库根)"; exit 1; }

python - "$LB" <<'PY'
import sys, io, os
LB = sys.argv[1]

def patch(path, old, new, marker):
    p = os.path.join(LB, path)
    s = io.open(p, encoding="utf-8").read()
    if marker in s:
        print("  skip (已打)", path); return
    if old not in s:
        print("  !! 未找到锚点,手动检查", path); return
    io.open(p, "w", encoding="utf-8").write(s.replace(old, new, 1))
    print("  patched", path)

# 1) 编码:from_ndarray → 构造器
patch("datasets/depth_utils.py",
'''    if video_backend == "pyav":
        frame = av.VideoFrame.from_ndarray(quantized, format=pix_fmt)
        write_u16_plane(frame.planes[0], quantized)
        return frame''',
'''    if video_backend == "pyav":
        # Some pyav builds lack numpy<->gray12le conversion; build via constructor + write_u16_plane.
        h, w = quantized.shape[:2]
        frame = av.VideoFrame(w, h, pix_fmt)
        write_u16_plane(frame.planes[0], quantized)
        return frame''',
    marker="av.VideoFrame(w, h, pix_fmt)")

# 2) 解码:to_ndarray(gray12le) → 手动读 plane
patch("datasets/video_utils.py",
'''            if is_depth:
                arr = frame.to_ndarray(format="gray12le")  # (H, W) uint12
                loaded_frames.append(torch.from_numpy(arr).unsqueeze(0).contiguous())''',
'''            if is_depth:
                # Some pyav builds lack to_ndarray(gray12le); read the u16 plane directly.
                _pl = frame.planes[0]
                _buf = np.frombuffer(bytes(_pl), dtype=np.uint16)
                arr = _buf.reshape(frame.height, _pl.line_size // 2)[:, : frame.width]
                loaded_frames.append(torch.from_numpy(arr.copy()).unsqueeze(0).contiguous())''',
    marker="_pl.line_size // 2")
PY
echo "[depthfix] 完成。带深度录制若之前崩在 save_episode/gray12le,现在应正常。"
