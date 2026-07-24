#!/usr/bin/env python3
"""红色阈值现场标定工具（G 题）。

火源改成印刷图案后，颜色阈值**必须在测试现场的实际光照下标**——
题目明说会有顶部照明和窗外自然光、光照不均，实验室标好的值搬到现场大概率不能用。

用法（车上/机上接显示器或 VNC，不需要飞，把相机举到 18dm 左右对着图案）：

    ros2 run fire_vision_pkg fire_color_tune

拖滑块直到：**只有火花图案是白的，底布/坐标线/街区/红色停车区全黑**。
按 `p` 打印当前参数，直接粘进 fire_mission.launch.py。按 `q` 退出。

窗口说明：
  original  原图，绿框=通过全部过滤的候选，红框=颜色过了但形状/面积没过
  mask      二值掩膜，就看这个调
"""

from __future__ import annotations

import argparse

import cv2
import numpy as np

WIN = "fire_color_tune"
MASK_WIN = "mask"


def nothing(_):
    pass


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0",
                    help="相机设备，建议用 /dev/v4l/by-path/... 稳定路径")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--rotate", type=int, default=2,
                    help="-1=不转 0=顺90 1=180 2=逆90（下视相机通常 2）")
    ap.add_argument("--no-lock-wb", action="store_true",
                    help="不锁白平衡（默认锁，与 fire_detector 行为一致）")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"打不开相机 {args.device}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not args.no_lock_wb:
        # 必须和 fire_detector 一致，否则这里标的值拿过去不成立
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        if cap.get(cv2.CAP_PROP_AUTO_WB) != 0:
            print("⚠ 白平衡锁定失败（驱动不支持）。红色会随光照漂，"
                  "标出来的阈值稳定性存疑，建议多换几个角度确认。")

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("lab_a_min", WIN, 150, 255, nothing)
    cv2.createTrackbar("hsv_s_min", WIN, 90, 255, nothing)
    cv2.createTrackbar("hsv_v_min", WIN, 50, 255, nothing)
    cv2.createTrackbar("morph", WIN, 5, 15, nothing)
    cv2.createTrackbar("min_area", WIN, 60, 2000, nothing)
    cv2.createTrackbar("max_area/100", WIN, 60, 500, nothing)

    print(__doc__)
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        if args.rotate in (0, 1, 2):
            frame = cv2.rotate(frame, args.rotate)

        a_min = cv2.getTrackbarPos("lab_a_min", WIN)
        s_min = cv2.getTrackbarPos("hsv_s_min", WIN)
        v_min = cv2.getTrackbarPos("hsv_v_min", WIN)
        k_size = max(1, cv2.getTrackbarPos("morph", WIN))
        min_area = cv2.getTrackbarPos("min_area", WIN)
        max_area = cv2.getTrackbarPos("max_area/100", WIN) * 100

        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = ((lab[:, :, 1] >= a_min) &
                (hsv[:, :, 1] >= s_min) &
                (hsv[:, :, 2] >= v_min)).astype(np.uint8) * 255
        k = np.ones((k_size, k_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        vis = frame.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        passed = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < 10:
                continue
            x, y, w, h = cv2.boundingRect(c)
            ok_area = min_area <= area <= max_area
            fill = area / float(w * h) if w and h else 0.0
            ok_shape = fill >= 0.25 and max(w / max(h, 1), h / max(w, 1)) <= 4.0
            good = ok_area and ok_shape
            passed += int(good)
            cv2.rectangle(vis, (x, y), (x + w, y + h),
                          (0, 255, 0) if good else (0, 0, 255), 2)
            cv2.putText(vis, f"{int(area)} f={fill:.2f}", (x, max(12, y - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (0, 255, 0) if good else (0, 0, 255), 1)

        cv2.putText(vis, f"pass={passed}  (期望只有 1 个绿框)", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow(WIN, vis)
        cv2.imshow(MASK_WIN, mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        if key == ord("p"):
            print("\n──── 粘进 fire_mission.launch.py 的 fire_detector 参数 ────")
            print(f"'lab_a_min': {a_min},")
            print(f"'hsv_s_min': {s_min},")
            print(f"'hsv_v_min': {v_min},")
            print(f"'morph_kernel': {k_size},")
            print(f"'min_area_px': {min_area},")
            print(f"'max_area_px': {max_area},")
            print("──────────────────────────────────────────────────────\n")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
