# Gemini 335 自动桌面标定

运行（关闭其他相机客户端，固定相机和桌面板）：

```bash
cd /home/descfly/ty/replay/code/calibrate
.venv/bin/python observe_orbbec.py --show
```

默认选择 CP0B163000CK，1280×720 MJPG / 10 fps。直接从所选流读取内参，
使用 8 项畸变系数 k1,k2,p1,p2,k3,k4,k5,k6，不需要手动拍照或导入内参。
等待曝光稳定后自动收集 30 帧；通过角点、重投影和位姿稳定性检查后保存并退出。
预览窗口按 q 或 Esc 取消。需要人工决定保存时机时加 --manual-save，稳定后按 s。
无显示器时不加 --show。默认超时 600 秒。

板参数与参考 Franka 工程 RViz 配置一致：14×9 格，格长 20 mm，标记边长 15 mm，
DICT_5X5_250，legacy pattern。图案必须与 board/board_reference.png 一致。

## 与参考工程的对应关系

对照仓库：/home/descfly/ty/replay/eye_hand/franka_orbbec_handeye_calibration。
已读取其固定版本 moveit_calibration 子模块 fbf21aa7c76258f18966fd68f555144073889caf。
检测参考文件为 handeye_calibration_target/src/handeye_target_charuco.cpp：

1. detectMarkers，CORNER_REFINE_NONE。
2. interpolateCornersCharuco，使用真实 K/D。
3. estimatePoseCharucoBoard，求相机中的板位姿。

当前实时入口默认使用该流程；原照片/USB/ROS 入口的 robust 算法保持原默认值。
保留额外质量门槛：至少 12 个角点、足够的板面覆盖、RMS <= 1.5 px、
30 帧最大位置散布 <= 3 mm、角度散布 <= 1 度。
原工程的机器人多姿态手眼求解和历史 Franka 外参不用于桌面标定。

## 桌面定义和结果

板应平行于桌面。默认认为图案面与桌面共面。
如果图案面在桌面上方 3 mm（包含硬板厚度），使用：

```bash
.venv/bin/python observe_orbbec.py --show --board-height-mm 3
```

输出到 results/arx_x5_table_时间戳/，不覆盖旧结果：

- observation.json：camera_T_board、board_T_camera、相机内参和质量统计。
- table_calibration.json：table_T_camera、camera_T_table、用户提供的板高及适用假设。
- image.png、overlay.jpg、samples.json：原图、叠加图和每帧观测。

A_T_B 将 B 坐标转换到 A 坐标，米、列向量。
桌面原点是图案左上外角在桌面上的垂直投影，X/Y 沿图案向右/向下，Z 指向桌内。
不是以前人工测量的“右/前/上”坐标系，也不是机器人基座系。
table_T_camera = table_T_board @ board_T_camera，其中 table_T_board 的 Z 平移是负板高。
板高由用户提供，本程序不会测量板厚。标定板倾斜时此桌面换算不成立。
机械臂基座外参保持未知；输出明确标记 complete_hand_eye_calibration=false。

## 验证范围

已在两份合成已知位姿图像上验证新检测流程、无板/尺寸错误拒绝和稳定窗口判断。
合成结果不等于真实绝对精度。真实结果应检查 overlay.jpg，并使用板外已知点独立复核。
只验证出流可运行 --check-stream --timeout 15，此模式不保存标定。

本机运行环境复用了 cam/.venv 内相同 CPython 3.12 的 pyorbbecsdk2 2.1.3 包，
配合标定环境已有的 numpy / opencv-contrib-python / PyYAML。
