# RTK-funny

# RTK-Aided LiDAR Mapping & Localization (ROS 2 + Livox MID-360)

End-to-end procedure for building a map, grading it with RTK/GNSS, and localizing against it.
Written for ROS 2 Humble on Jetson (ARM64), verified on a `funny_lidar_slam` / `qc_lidar_slam` tree.

---

## 0. Pipeline at a glance

```
Livox MID-360 ──► livox_ros_driver2 ──► /livox/lidar (PointCloud2)  ─┐
                                       /livox/imu                  ─┤
RTK receiver ──► rtk_gnss_publisher ──► /gnss/fix, /gnss/velocity, ─┤
                                        /gnss/heading               │
                                                                    ▼
                                                    funny_lidar_slam (mapping)
                                                                    │
                                        keyframes/ keyframe_poses.txt enu_origin.txt
                                        map.pcd mapping_meta.txt  ... │
                                                                    ▼
                                            rtk_map_optimizer (offline, Ceres)
                                                                    │
                                       optimized_keyframe_poses.txt map_enu.pcd
                                                                    │
                                                                    ▼
                                              localization (map.pcd / map_enu.pcd)
```

Rule of thumb: **mapping is online, RTK optimization is offline, localization is online again.** Never run mapping and optimization at the same time.

---

## 1. Network: fix the addresses once

The MID-360 is a static device. In this setup it sends to `192.168.168.135`, and it does so **even when no driver is running** — that is how you identify it.

```bash
# which interface carries the lidar
ip -br addr

# add the host address the lidar expects (temporary!)
sudo ip addr add 192.168.168.135/24 dev enP8p1s0

# proof the link is alive: ARP + UDP, no driver needed
sudo tcpdump -ni enP8p1s0 -c 8 'arp or udp portrange 56300-56401'
```

Expected output:

```
ARP  Request who-has 192.168.168.135 tell 192.168.168.51
ARP  Reply 192.168.168.135 is-at <host mac>
UDP  192.168.168.51.56300 > 192.168.168.135.56301  length 1380   # point data
UDP  192.168.168.51.56400 > 192.168.168.135.56401  length 60     # IMU
```

**Make it permanent** — `ip addr add` disappears on reboot and the driver then fails with `bind failed`:

```bash
nmcli -g ipv4.method,ipv4.addresses connection show "Wired connection 1"

sudo nmcli connection modify "Wired connection 1" \
  ipv4.addresses "192.168.123.135/24,192.168.168.135/24"
sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up   "Wired connection 1"
```

> A lidar IP that is *not* `192.168.1.1XX` (factory = `192.168.1.1` + last two serial digits) was re-addressed by the integrator. Treat it as fixed; every config refers to it now.

---

## 2. Environment check

```bash
cd ~/livox_ws && source install/setup.bash
ros2 pkg executables livox_ros_driver2

cd ~/funny_rtk_slam_ws && source install/setup.bash
ros2 pkg executables funny_lidar_slam
ros2 interface show funny_lidar_slam/srv/SaveMap
```

Read the srv definition before composing any request — field sets differ between forks (upstream ROS 1 has 2 fields, ROS 2 forks often have 3).

---

## 3. Launch the lidar driver

```bash
cd ~/livox_ws && source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

Healthy startup:

```
Livox Ros Driver2 Version: 1.2.7
config lidar type: 8
successfully parse base config, counts: 1
Init lds lidar success!
successfully set lidar attitude, ip: 192.168.168.51
successfully enable Livox Lidar imu, ip: 192.168.168.51
livox/lidar publish use PointCloud2 format
```

`bind failed` instead ⇒ the host address is missing, not that the build is broken. Check:

```bash
ip -4 addr show enP8p1s0 | grep 192.168.168.135
ss -lunp | grep -E ':56301|:56401'
```

Empty address = `EADDRNOTAVAIL` (fix the address). Ports already held = `EADDRINUSE` (kill the orphan: `sudo pkill -9 -f livox_ros_driver2_node`).

Verify data before touching SLAM:

```bash
ros2 topic hz /livox/lidar     # ~10 Hz
ros2 topic hz /livox/imu
```

---

## 4. Launch the RTK / GNSS publisher

```bash
cd ~/rtk_ws && source install/setup.bash
ros2 launch rtk_gnss_publisher rtk_gnss.launch.py
```

Healthy startup:

```
Opening serial port: /dev/ttyUSB0 @ 115200 baud
```

```bash
ros2 topic hz /gnss/fix
ros2 topic echo /gnss/fix --once --field status
ros2 topic echo /gnss/fix --once --field position_covariance
ros2 topic echo /gnss/heading --once
```

Quality reference (NMEA GGA quality indicator):

| Value | Meaning | Suitable for map grading |
|---|---|---|
| 4 | RTK **fixed** (cm) | yes |
| 5 | RTK **float** (dm) | marginal |
| 2 | DGPS | no |
| 0 / 1 | no fix / standalone | no |

`NavSatFix` has no `covariance` field — the array is `position_covariance`.

Serial permissions, if `/dev/ttyUSB0` cannot be opened:

```bash
sudo usermod -aG dialout $USER        # log out / in afterwards
ls -l /dev/ttyUSB0
```

---

## 5. Mapping (enable artifact saving)

**Check that mapping actually writes its artifacts** — this is the step that silently ruins later optimization:

```bash
cd ~/funny_rtk_slam_ws/src/qc_lidar_slam
grep -rn "save_mapping_path" config/ launch/
```

Set `save_mapping_path: 1` (or the equivalent flag in your fork). With `0`, only `map.pcd` is written and there is no `keyframe_poses.txt` / `enu_origin.txt` for the optimizer to consume.

```bash
cd ~/funny_rtk_slam_ws && source install/setup.bash
ros2 launch funny_lidar_slam mapping_mid360.launch.py
```

Operational notes:

- Keep the robot **still for ≥2 s** at start so the IMU initializes.
- Drive slowly on the first pass; fast motion + 10 Hz LiDAR degrades the pose graph.
- Watch for queue warnings during startup — a few are normal, continuous ones mean the CPU is saturated.
- Wait for `Initial mapping successfully` / keyframes accumulating before trusting anything.

---

## 6. Save the map (with an explicit path)

```bash
ros2 interface show funny_lidar_slam/srv/SaveMap     # read the real field set
ros2 service list | grep -i save_map
```

3-field form:

```bash
wheeltec@wheeltec:~/funny_rtk_slam_ws$ ros2 service call /funny_lidar_slam/save_map funny_lidar_slam/srv/SaveMap  "{map_path: '', split_map: false, map_voxel_size: 0.1}"

```

2-field form: drop `map_voxel_size`.

- `map_path: ''` writes into the package `data/` directory and **will be overwritten by the next empty-path call**. Always give a path.
- `map_voxel_size: 0.0` = no downsampling; a large session produces a huge file and the call looks frozen. Use `0.05`–`0.1` if disk is tight.

---

# Create the bag2 directory

    mkdir -p ~/rtk_bag2
    cd ~/rtk_bag2

# Record all required topics for RTK mapping

    ros2 bag record --storage mcap --max-bag-size 2000000000 \
    --output rtk_bag2_$(date +%Y%m%d_%H%M%S) \
    /livox/lidar \
    /livox/imu \
    /gnss/fix \
    /gnss/heading \
    /gnss/velocity \
    /tf \
    /tf_static

## 7. Stop mapping, then verify the artifact contract

```bash
# Ctrl-C in the mapping terminal
ls -la ~/funny_rtk_slam_ws/src/qc_lidar_slam/data/
```

| File / dir | Produced by | Purpose |
|---|---|---|
| `keyframes/` | mapping | per-keyframe clouds/poses; empty ⇒ nothing to optimize |
| `keyframe_poses.txt` | mapping | pose trajectory the optimizer adjusts |
| `map.pcd` | mapping | map in the SLAM frame |
| `mapping_meta.txt` | mapping | session metadata |
| `enu_origin.txt` | mapping | lat/lon/alt of the ENU origin — the geo-reference |
| `loop_closure_edges.txt` | mapping | may be **0 bytes** if the path never revisited; then RTK is the only global constraint |
| `rtk/`, `rtk_data/` | input prep | RTK track used by the optimization |
| `optimized_keyframe_poses.txt` | **optimization** | corrected poses |
| `map_enu.pcd` | **optimization** | map re-expressed in ENU |
| `rtk_trajectory.png` | visualization | plot |

If `keyframes/` or `keyframe_poses.txt` is missing, stop here — optimization cannot run.

---

## 8. RTK optimization

Config: `src/qc_lidar_slam/config/optimization/rtk_optimize.yaml`

```yaml
rtk_optimize:
  data_dir: "/home/user/ws/src/qc_lidar_slam/data"
  lever_arm: [-0.080, -0.005, 0.065]      # antenna offset from the LiDAR, metres
  time_offset: 0.0                        # GNSS → LiDAR time offset, seconds
  heading_ned: true                       # input heading in NED (clockwise from north)
  heading_yaw_offset: -3.14159            # rotate into the SLAM frame
  rtk_position_noise: [0.05, 0.05, 0.10]  # x, y, z sigma
  heading_noise_deg: 1.5
  fix_status_min: 2                       # minimum accepted fix quality
  fix_cov_max: 0.09                       # max accepted position covariance
  use_heading: true
  output_voxel_size: 0.3
  output_map_path: ""
```

Run from the workspace root with an **absolute** yaml path (offset style, so a missing argument cannot silently become `""`):

```bash
cd ~/funny_rtk_slam_ws
source install/setup.bash

ros2 run funny_lidar_slam rtk_map_optimizer \
  /home/wheeltec/funny_rtk_slam_ws/src/qc_lidar_slam/config/optimization/rtk_optimize.yaml
```

`YAML::BadFile ... bad file:` with an empty name means the node received an empty path — usually a failed `$(find ...)` substitution. Use a literal absolute path.

### Reading the solver output

```
Built 104 RTK observations from 153 keyframes.
Initial cost: 9.359299e+03, Final cost: 8.876778e+03, Termination: NO_CONVERGENCE
```

| Signal | Healthy | Suspicious |
|---|---|---|
| Termination | `CONVERGENCE`, <20 iterations | `NO_CONVERGENCE` at the iteration cap |
| Cost drop | 10×–100× | a few percent |
| Cost / observation | ≲ 9 (≈3σ) | ≫ 9 (systematic geometry error) |
| Coverage | fewer observations is fine | wrong-but-confident fixes are worse than missing ones |

Cost per observation is the most informative number: with `rtk_position_noise = 0.05 m`, a cost of ~90 per fix implies ~0.5 m of systematic disagreement — that is a convention error, not noise.

### Isolation experiments (each run is ~2 s)

**Heading convention** — disable the heading factor:

```bash
cd ~/funny_rtk_slam_ws/src/qc_lidar_slam/config/optimization
cp rtk_optimize.yaml rtk_optimize_a.yaml
sed -i 's/use_heading: true/use_heading: false/' rtk_optimize_a.yaml
```

Converges ⇒ the `heading_ned` / `heading_yaw_offset` combination is wrong. Re-enable heading and test `heading_yaw_offset: 0.0`, then `heading_ned: false`.

**Indoor multipath** — accept RTK-fixed solutions only:

```bash
cp rtk_optimize.yaml rtk_optimize_b.yaml
sed -i 's/use_heading: true/use_heading: false/' rtk_optimize_b.yaml
sed -i 's/fix_status_min: 2/fix_status_min: 4/' rtk_optimize_b.yaml
```

B converges while A does not ⇒ indoor float fixes were poisoning the graph. Keep the stricter gate and accept lower coverage.

```bash
ros2 run funny_lidar_slam rtk_map_optimizer \
  $(realpath src/qc_lidar_slam/config/optimization/rtk_optimize_a.yaml)
ros2 run funny_lidar_slam rtk_map_optimizer \
  $(realpath src/qc_lidar_slam/config/optimization/rtk_optimize_b.yaml)
```

# Acceptance before using `map_enu.pcd`

1. `Termination: CONVERGENCE`
2. final cost ≥ 10× below initial
3. cost per observation ≲ 9

Fail any of them and `map_enu.pcd` is just the SLAM map with a weak rigid transform — plausible in RViz, wrong in metres.

---

## 9. Visualize the graded trajectory

```bash
cd ~/funny_rtk_slam_ws && source install/setup.bash
python3 "$(find src -name 'plot_rtk_trajectory.py' | head -1)" --help
```

Run `--help` first: the argument set differs between forks. It normally expects the optimized trajectory plus the raw RTK track, and writes `rtk_trajectory.png`.

---

## 10. Localization

**Choose the map deliberately:**

| File | Frame | Use when |
|---|---|---|
| `data/map.pcd` | SLAM frame | indoor / local work; unaffected by the RTK solve |
| `data/map_enu.pcd` | ENU (0.3 m voxel) | you need global East-North coordinates, **only after a converged optimization** |

```bash
pgrep -af livox_ros_driver2_node
ros2 topic hz /livox/lidar
ls -lh ~/funny_rtk_slam_ws/src/qc_lidar_slam/data/map.pcd

cd ~/funny_rtk_slam_ws && source install/setup.bash
ros2 launch funny_lidar_slam localization_mid360.launch.py
```

Then, in RViz, publish `2D Pose Estimate` at the robot's approximate position and heading.

- This tree **converges from a prior; it does not search globally.** A wrong initial guess will not recover on its own — re-issue it.
- Check that the launch actually loads `data/map.pcd` and not a copy inside `install/.../share/...`.

Verify the lock from TF, not from the point cloud:

```bash
ros2 run tf2_ros tf2_echo map body
```

| Behaviour | Meaning | Action |
|---|---|---|
| ± few cm jitter while still | good lock | proceed |
| steady creep in one direction | wrong prior | re-issue `2D Pose Estimate` |
| no `map→body` at all | node exited at startup | check map path and frame id |

---

## 11. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `bind failed` at driver start | host address absent (`EADDRNOTAVAIL`) | add `192.168.168.135/24`; persist it with `nmcli` |
| `bind failed` with address present | port already held (`EADDRINUSE`) | `sudo pkill -9 -f livox_ros_driver2_node` |
| `ping <lidar>` works, no data | ICMP resolves on any address in the subnet; the stream needs the exact address | make the host address equal the lidar's target |
| `YAML::BadFile ... bad file:` | node received an empty/relative path | pass a literal absolute path |
| optimizer exits instantly | yaml sample paths still present, or `data_dir` empty | absolutize paths; verify `keyframes/` |
| `NO_CONVERGENCE`, cost −5 % | conflicting constraints: heading convention or bad fixes | run the A/B experiments |
| fixes silently ignored | `position_covariance[0] > fix_cov_max` | measure the real covariance first |
| `loop_closure_edges.txt` 0 bytes | path never revisited | expected; RTK is then the only global constraint |
| localization never locks | no global relocalization in this tree | set the initial pose manually |
| `package not found` | ran from `src/`, or workspace not sourced | run at workspace root after `source install/setup.bash` |

---

## 12. Things this pipeline does *not* do

- No automatic **global relocalization** — the initial pose is an input, not a search.
- RTK grading inherits the fix quality: a float-only session yields a decimetre map, and no loop-closure factor will rescue it.
- A single graph spanning indoor and outdoor inherits the worst fixes from both; prefer one session per environment.
- `optimized_keyframe_poses.txt` contains corrected poses only; do not expect extra information beyond index, timestamp and pose.

> （注：部分内容由豆包 AI 生成）
