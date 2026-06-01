# qr_vision_pkg

D题 立体货架盘点 — 单相机二维码识别 + 激光指示 + 对准微调。
移植自 24fly `opencv01`，按"单相机 + yaw 旋转扫面"方案改造（去掉左右相机切换，加 `rotate_code`
方位适配、`/qr_vision/enable` 门控）。

## 节点
- `qr_vision`（`decoder_common.py`）：识别二维码，发布 `/qr_vision/{id,offset_norm,aligned,debug_image}`；
  到位对准后子线程打激光 0.5s。订阅 `/qr_vision/enable` 总开关。
- `qr_fine_tune`（`qr_fine_tune.py`）：归一化偏移 → 机体系 cm 微调，发布 `/qr_vision/fine_offset_body_cm`。

## 关键参数
`camera_device`、`rotate_code`(-1/0/1/2)、`fourcc`(默认 MJPG)、`laser_pin`(wiringPi，-1=不控)、
`eps_x`/`eps_x_laser`/`stable_frames`/`decode_interval`。

详见仓库根 `立体货架盘点无人机_开发笔记.md`。
