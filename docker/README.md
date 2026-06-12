# TagSLAM Checkerboard Marker Baseline — 运行文档

用 **TagSLAM** 作为 marker 基线，在 ov_plane 的 **Table** 数据集（棋盘标定板，间歇可见）上产出轨迹，并与 OpenVINS 轨迹拼接成连续全程轨迹，用于和其它方法做 ATE 对比。整个流程在 **Docker** 中编译运行，本机只需安装 Docker，不污染本机 ROS 环境。

---

## 1. 原理与数据流

TagSLAM 只接受 `apriltag_msgs/AprilTagDetectionArray`，而 Table 数据用的是棋盘。因此先把棋盘离线检测成「虚拟 AprilTag」，再喂给 TagSLAM；棋盘不可见时由 OpenVINS odom 维持连续。

```
table_0X.db3 ──/d455/color/image_raw──▶ make_tag_bag.py ──┐ (离线逐帧检测棋盘→虚拟tag,
            └─/d455/rigidbody (mocap GT)                   │  OpenVINS姿态消歧, 无丢帧)
                                                           ▼
Results/.../table0X.txt ─▶ tum_to_odom_bag.py ─▶ odom_0X ──┤
                                                           ▼
                                          merge_bags.py (按时间戳对齐合并)
                                                           ▼
                                   tagslam_node (cameras/camera_poses/tagslam.yaml)
                                          订阅 /detector/tags + /ov_msckf/odomimu
                                                           ▼ 发布 /odom/body_rig
                                          录制 → bag_topic_to_tum.py → tagslam_0X.txt
                                                           ▼
                                   stitch_trajectory.py (棋盘可见段用tagslam,
                                          不可见段用对齐后的OpenVINS) → stitched_0X.txt
                                                           ▼
                                          evo_ape / ov_eval 对 GT 算 ATE
```

---

## 2. 前置条件

- 本机已安装 Docker，且当前用户免 sudo（`docker ps` 不报权限错误）。
- 数据就位：
  - 数据集 bag：`/home/lee/Documents/Datasets_VSLAM/table/table_0X/`（ROS2 bag）
  - OpenVINS 轨迹（TUM）：`/home/lee/Results/board/openvins/table/table0X.txt`
  - GT（任选其一）：
    - RPNG 官方 GT（与表3 同口径，**推荐**）：`/home/lee/vio_ws/.../ov_data/rpng_plane/table_0X.txt`
    - 或 bag 内 mocap：`/d455/rigidbody`（用 `bag_topic_to_tum.py` 提取）

---

## 3. 快速开始

```bash
# (1) 构建镜像并进入容器（首次会装 GTSAM/apriltag/flex_sync 等，约几分钟）
bash /home/lee/tagslam/docker/run.sh

# —— 以下命令在容器内 ——

# (2) 首次编译 tagslam（产物落在挂载卷 /opt/tagslam_ws -> 主机 .docker_ws/）
bash /home/lee/tagslam/docker/build_ws.sh

# (3) 跑单个序列（含离线检测+合并+tagslam+提取轨迹）
bash /home/lee/tagslam/scripts/run_tagslam_table.sh 01

# (4) 拼接成连续全程轨迹
python3 /home/lee/tagslam/scripts/stitch_trajectory.py \
  /home/lee/tagslam/table_eval/traj/tagslam_01.txt \
  /home/lee/Results/board/openvins/table/table01.txt \
  /home/lee/tagslam/table_eval/traj/stitched_01.txt

# (5) 评估（RPNG GT）
evo_ape tum \
  /home/lee/vio_ws/src/vio-gnss/ekf-localization/examples/ros1/src/ov_data/rpng_plane/table_01.txt \
  /home/lee/tagslam/table_eval/traj/stitched_01.txt -a
```

> 注意：`run.sh` 默认未挂载 `vio_ws`（RPNG GT）。若要在容器内用 RPNG GT 评估，在 `run.sh` 的 `docker run` 里加 `-v /home/lee/vio_ws:/home/lee/vio_ws:ro`，或在主机用你自己的 ov_eval 评估。

---

## 4. 分步详解

### 4.1 构建镜像
`docker/run.sh` 会在镜像不存在时自动 `docker build`（用 `docker/Dockerfile`），然后挂载所有路径并进入交互 bash。镜像通过 `rosdep` 把 tagslam 的全部依赖（GTSAM、apriltag_msgs、apriltag_detector、flex_sync、cv_bridge 等）装进镜像；numpy 钉在 1.x 以兼容 cv_bridge。

挂载关系（容器内 = 主机绝对路径，便于脚本路径一致）：

| 容器内 | 主机 | 权限 |
|---|---|---|
| `/home/lee/tagslam` | 项目源码 | rw |
| `/opt/tagslam_ws` | `/home/lee/tagslam/.docker_ws` | rw（编译产物，已 gitignore）|
| `/home/lee/Documents/Datasets_VSLAM/table` | 数据集 | ro |
| `/home/lee/Results/board/openvins/table` | OpenVINS 轨迹 | ro |

### 4.2 编译 tagslam
`docker/build_ws.sh`（容器内）把源码软链进 `/opt/tagslam_ws/src` 并 `colcon build --symlink-install`。产物持久在主机 `.docker_ws/`，重进容器无需重编。改 Python 脚本 / yaml 配置**不需要**重编（运行时读取）。

> 注：`CMakeLists.txt` 中 `tagslam_from_bag` / `sync_and_detect_from_bag` 两个 target 已用 `if(FALSE)` 禁用——它们依赖 Humble 没有的 rosbag2 API，且本流程用标准 `ros2 bag play`，不需要。

### 4.3 准备 odom bag（已生成）
`tum_to_odom_bag.py` 把 OpenVINS 的 TUM 轨迹转成 `nav_msgs/Odometry` bag（topic `/ov_msckf/odomimu`，`frame_id=odom`，`child_frame_id=imu`）。5 个序列已生成在 `table_eval/odom/`。重新生成：
```bash
python3 scripts/tum_to_odom_bag.py \
  /home/lee/Results/board/openvins/table/table01.txt \
  table_eval/odom/odom_01 --topic /ov_msckf/odomimu --frame odom --child-frame imu
```

### 4.4 准备 GT
`bag_topic_to_tum.py` 从 ROS2 bag 提取位姿 topic 转 TUM：
```bash
python3 scripts/bag_topic_to_tum.py \
  /home/lee/Documents/Datasets_VSLAM/table/table_01 /d455/rigidbody table_eval/gt/gt_01.txt
```
**推荐评估用 RPNG 官方 GT**（与表3 同口径），即 `rpng_plane/table_0X.txt`，无需提取。

### 4.5 跑单序列
`scripts/run_tagslam_table.sh <NN>` 四步：
1. **离线生成 tag bag**（`make_tag_bag.py`，逐帧检测棋盘→虚拟 tag，慢，约 6 分钟/序列，缓存在 `table_eval/tags/tag_NN`）；
2. 启动 `tagslam_node` + 录制 `/odom/body_rig`；
3. `merge_bags.py` 把 tag bag 和 odom bag **按时间戳对齐合并**，单次 `ros2 bag play --clock` 回放；
4. 提取 rig 轨迹到 `table_eval/traj/tagslam_NN.txt`。

### 4.6 批量
```bash
for i in 01 02 03 04 05; do
  bash scripts/run_tagslam_table.sh $i
  python3 scripts/stitch_trajectory.py \
    table_eval/traj/tagslam_$i.txt \
    /home/lee/Results/board/openvins/table/table$i.txt \
    table_eval/traj/stitched_$i.txt
done
```

### 4.7 评估
- evo（容器内已装）：`evo_ape tum <GT> <traj> -a`（`-a` = SE3 对齐；`-as` 加尺度）
- 或 ov_eval（你的 OpenVINS 环境）：`ros2 run ov_eval error_singlerun sim3 <GT> <traj>`

---

## 5. 脚本清单（`scripts/`）

| 脚本 | 作用 |
|---|---|
| `make_tag_bag.py` | 离线逐帧检测棋盘→虚拟 tag bag；含 OpenVINS 姿态消歧 + IPPE 双解 + 高置信过滤 |
| `tum_to_odom_bag.py` | OpenVINS TUM 轨迹 → `nav_msgs/Odometry` bag |
| `merge_bags.py` | 多个 bag 按时间戳合并成一个（时间对齐回放）|
| `bag_topic_to_tum.py` | 从 bag 提取 PoseStamped/Odometry topic → TUM |
| `stitch_trajectory.py` | tagslam 段（棋盘可见）+ OpenVINS（不可见段，SE3 对齐到棋盘系）拼接 |
| `run_tagslam_table.sh` | 单序列一键流程（1~4 步）|
| `check_board_visibility.py` | 离线统计棋盘在某 bag 中的可见帧数与时间分布（诊断用）|
| `checkerboard_to_apriltag.py` | 实时桥接节点（已弃用，回放会丢帧，改用离线 `make_tag_bag.py`）|

---

## 6. 配置文件（`config/table/`）

- `cameras.yaml`：相机内参/畸变/topic。**用 bag 真实内参**（848×480，fx≈419，plumb_bob），不是 OpenVINS 的 kalibr 标定（那是 D435i 的 640×480/fx≈586，与本数据不符）。
- `camera_poses.yaml`：相机相对 rig(IMU) 外参 = kalibr `T_imu_cam`（tagslam 内部 `T_r_c` 即「相机在 rig 系的位姿」，与之一致）。
- `tagslam.yaml`：静态 `board` body（4 个虚拟 tag 的已知 pose）+ 非静态 `rig` body（接 `/ov_msckf/odomimu`，`odom_frame_id: imu`）。

---

## 7. 关键注意事项（踩过的坑）

1. **内参**：必须用 bag 的 `/d455/color/camera_info`（848×480, fx≈419），用错内参 tag 投影全错。
2. **odom frame**：odom 的 `child_frame_id` 与 `tagslam.yaml` 的 `odom_frame_id` 必须都是 `imu`，否则 tagslam 报 “no body found for odom frame id”。
3. **输出 topic**：tagslam 实时发布的是 `/odom/body_rig`（**无** `/tagslam` 前缀；带前缀的名字只用于它自己写 bag 的模式）。
4. **时间对齐**：tag bag 和 odom bag 必须用 `merge_bags.py` 合并后单次回放；两个独立 `ros2 bag play` 起始时间戳不同会导致 tag/odom 同步错位。
5. **文件属主**：容器以 root 写文件，主机删/改前需 `chown`。批量脚本末尾会自动 `chown -R 1000:1000 table_eval`。
6. **缓存**：`table_eval/tags/tag_NN` 是 `make_tag_bag` 的缓存；改了 `make_tag_bag.py` 要先删缓存才会重新生成。

---

## 8. 已知局限（重要）

**棋盘（6×6 对称 + 共面）做纯视觉 TagSLAM 在 Table 数据上无法产出准确全程轨迹**，全程 ATE 约 1m 量级（连续可见短段可达 ~5mm）。根因是棋盘的两重单帧不可消歧义：

- **180° 旋转对称**：6×6 棋盘绕法向转 180° 投影完全相同；
- **共面 IPPE 倾斜歧义**：平面 PnP 的两个倾斜解。

在 Table 的「运动平缓 + 近正对」下，OpenVINS 姿态无法在所有帧可靠区分它们，残余错误帧把因子图（和准确的 odom）拉偏，表现为方向系统性错（`rmse_ori≈166°`）、轨迹长度膨胀。这不是配置 bug，而是 checkerboard 纯视觉 marker SLAM 的固有局限——需要 IMU 紧耦合（如本项目对比的方法）才能消歧。

**本流程适用于**：场景中有真正的 AprilTag，或棋盘**持续可见**的序列。届时直接复用上述脚本即可。

---

## 9. 目录结构

```
/home/lee/tagslam/
├── docker/
│   ├── Dockerfile        # 运行时镜像（依赖固化）
│   ├── run.sh            # 构建镜像 + 进容器（挂载）
│   ├── build_ws.sh       # 容器内编译 tagslam
│   └── README.md         # 本文档
├── scripts/              # 见第 5 节
├── config/table/         # cameras / camera_poses / tagslam.yaml
├── table_eval/           # 产物（已 gitignore）
│   ├── odom/   odom_0X            # OpenVINS 轨迹转的 odom bag
│   ├── tags/   tag_0X, merged_0X  # 虚拟 tag bag + 合并 bag
│   ├── gt/     gt_0X.txt          # mocap GT（TUM）
│   ├── traj/   tagslam_0X.txt, stitched_0X.txt
│   ├── rec/    rec_0X             # 录制的 /odom/body_rig
│   └── log/    *.log
└── .docker_ws/           # 容器内编译产物（已 gitignore）
```
