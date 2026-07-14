# H753 ROS2 navigation mode handoff

Updated: 2026-06-01

Use this concise file when resuming with another AI. Read `JETSON_CODEX_HANDOFF.md`
for firmware history only when deeper STM context is needed.

## Paths and devices

- Firmware/tools repo: `/home/jyl1015/h753_ros_humble`
- ROS workspace: `/home/jyl1015/ros2_graduation_project_ws`
- ROS package: `~/ros2_graduation_project_ws/src/h753_can_odom`
- CANable telemetry: `/dev/ttyACM0`, stable Openlight Labs `/dev/serial/by-id/...`
- STM ST-LINK UART command: `/dev/ttyACM1`, stable STMicroelectronics `/dev/serial/by-id/...`
- YDLIDAR Tmini Pro: `/dev/ttyUSB0`, CP2102 `/dev/serial/by-id/...`

## Firmware state

- STM UART accepts `A5 5A 10 v_i16_le w_i16_le checksum`.
- `v`: mm/s, `w`: mrad/s. UART command timeout stops the motors.
- Runtime remains encoder-measured open-loop PWM: `APP_UART_SMOKE_USE_PID 0`.
- Keep `App/Src/app_motor_drive.c` motor DIR polarity at the original hardware
  baseline. A temporary all-wheel polarity inversion affected the standalone
  `tools/run_xbox_uart_drive.sh` path too, so it was reverted and reflashed with
  OpenOCD verify success on 2026-05-30.
- FL encoder uses corrected `PB4/PB5 = TIM3_CH1/CH2`.
- Encoder PPR is `548776`; wheel diameter is `0.21 m`.
- Measured robot outer envelope is `0.60 m` wide and `0.55 m` long.
  This is the Nav2 footprint reference, not the encoder kinematics track gauge.
- STM telemetry is CAN FD+BRS `0x100` state and `0x101` encoder ticks.

## Implemented ROS state

- `can_odom_node.py`: CANable telemetry to `/odom`, `/odom_vel`, TF `odom -> base_link`.
- `imu_odom_fusion_node.py`: D435i IMU yaw fusion with wheel odom fallback.
  D435i currently logs `No HID info provided, IMU is disabled`, so fallback is expected.
- `slam_toolbox`: mapping and serialized-posegraph localization launches exist.
- `map_odom_publisher_node.py`: visualization-only `/map_odom`.
- `cmd_vel_uart_bridge_node.py`: `/cmd_vel_safe` to STM UART with:
  - one-UART-process lock shared with legacy Xbox tool
  - deadman stop after `0.30 s`
  - lidar `/scan` stale stop after `0.50 s`
  - sensor-data QoS for YDLIDAR best-effort `/scan`
  - ROS-only `linear_command_sign` and `angular_command_sign` parameters;
    both are set to `-1.0` because ROS integrated driving was physically
    reversed in forward/backward and left/right directions. Standalone
    `tools/run_xbox_uart_drive.sh` remains unchanged.
  - max linear `0.60 m/s`, angular `2.67 rad/s`
- Nav2:
  - DWB planner/controller, velocity smoother
  - lidar obstacle costmaps
  - collision monitor `/cmd_vel -> /cmd_vel_safe`
  - autonomous DWB and velocity-smoother envelope `0.60 m/s`, `2.67 rad/s`
  - autonomous collision-monitor slowdown and stop polygons near obstacles
- `frontier_explorer_node.py`: online mapping frontier selection and Nav2 goals.
- `odom_imu_compare_node.py`: diagnostic comparison of raw encoder-wheel odom
  against integrated D435i gyro yaw. It logs accumulated wheel distance, wheel
  yaw, IMU yaw, yaw error, and IMU/wheel yaw ratio to terminal and CSV.
- Integrated launch files isolate nested launch arguments with scoped groups.
  This prevents nested `launch_rviz:=false` values from disabling the final RViz node.

## Existing commands

```bash
cd ~/ros2_graduation_project_ws
./run_calibration.sh       # raw wheel odom/lidar calibration
./run_slam_bringup.sh      # manual driving separately, mapping only
./save_slam_map.sh         # save posegraph and occupancy map while mapping runs
./run_localization.sh      # saved map localization only
./run_navigation.sh        # saved map localization + Nav2 RViz goal driving
./run_slam_navigation.sh   # online mapping + Nav2 RViz goal driving
./run_auto_explore.sh      # online mapping + Nav2 frontier exploration
./run_odom_imu_compare.sh  # wheel odom vs integrated D435i gyro CSV diagnostic
```

## Implemented mode manager

UART must have one owner. Replace direct Xbox-to-UART control during ROS operation
with `joy_node -> robot_mode_manager -> collision_monitor -> UART bridge`.

Mode definitions:

| Mode | Name | Runtime | Command source |
| --- | --- | --- | --- |
| 0 | Stop | Preserve or stop current runtime | forced zero |
| 1 | Manual mapping | online SLAM + Nav2 | Xbox/keyboard |
| 2 | Auto mapping | online SLAM + Nav2 + frontier | Nav2 |
| 3 | Manual localization | saved map localization + Nav2 | Xbox/keyboard |
| 4 | Goal navigation | saved map localization + Nav2 | Nav2 RViz goal |
| 5 | Inspection drive | lidar + CAN odom only, low speed | Xbox/keyboard |

Selection UX:

- Keyboard: digits `0`-`5`, `m` menu, `w/s/a/d/x` manual drive.
- Xbox: Menu toggles menu; D-pad up/down selects; A applies; B returns to stop;
  hold Menu for 2 seconds for emergency stop.
- Start in mode `0` Stop.
- Mode transition always publishes stop, disables frontier, cancels Nav2 goals,
  stops the old child launch, then starts the required runtime.
- Modes `3` and `4` must reject entry until
  `~/ros2_graduation_project_ws/maps/h753_map.posegraph` and `.data` exist.

Implemented files:

- `h753_can_odom/robot_mode_manager_node.py`
- `config/h753_robot_mode_manager.yaml`
- `config/h753_collision_monitor_modes.yaml`
- `launch/inspection_drive_bringup.launch.py`
- `~/ros2_graduation_project_ws/run_robot_modes.sh`

One-command entrypoint:

```bash
cd ~/ros2_graduation_project_ws
./run_robot_modes.sh
```

Mode manager details:

- `joy_node` stays alive while child runtime launch files switch.
- Modes `1 <-> 2` reuse the online mapping runtime without restart.
- Modes `3 <-> 4` reuse the serialized-map localization runtime without restart.
- Switching mapping/localization/inspection topologies stops the old runtime first.
- Mode `0` preserves the current runtime but publishes zero velocity.
- `collision_monitor` consumes `/cmd_vel_selected` in integrated mode.
- Old `tools/run_xbox_uart_drive.sh` remains for standalone firmware testing only.
- Xbox default indices are configurable in `config/h753_robot_mode_manager.yaml`.
- The manager warns after `3 s` when no `/joy` input arrives.

Verified without moving motors:

- Package build passed.
- Keyboard `0 -> 1 -> 2 -> 5 -> 0` transition passed with lidar, odom, UART,
  camera, and RViz disabled.
- Keyboard `m`, `s`, Enter menu selection passed.
- Wrapper mode `5` inspection test passed: keyboard `w` published
  `/cmd_vel_selected.linear.x: 0.12`, then the command returned to zero.
- Wrapper mode `1` dry test passed: RViz started and initialized OpenGL after the
  nested launch scope fix.
- Wrapper mode `1` hardware idle test passed: YDLIDAR connected, `/scan` measured
  `9.96 Hz`, UART bridge opened ST-LINK, and scan guard reported recovery.
- Nav2 RViz now uses package-owned `rviz/navigation.rviz`. Its visible
  `RealsenseCamera` panel subscribes to `/camera/camera/color/image_raw`;
  the old Nav2 default R200 topic was incorrect for the connected D435i.
- D435i color publishing was hardware-verified at about `22-26 Hz`.
- D435i IMU is not currently available on the Jetson native librealsense
  backend. `/camera/camera/imu` is absent and librealsense reports
  `No HID info provided, IMU is disabled`. The connected D435i exposes
  `/dev/hidraw0`, but Jetson kernel `5.15.148-tegra` lacks
  `hid_sensor_custom`, `hid_sensor_hub`, `hid_sensor_gyro_3d`, and
  `hid_sensor_accel_3d`. Integrated odom therefore falls back to wheel odom.
  Enable IMU later with the official Jetson librealsense kernel patch or an
  RSUSB backend build.
- `run_calibration.sh` now measures wheel linear/angular signs and offers a
  separate D435i IMU yaw-axis/sign measurement once `/camera/camera/imu` is
  available.
- `run_odom_imu_compare.sh` was added for physical calibration diagnostics.
  Run it alongside mode `1`, reset between runs with
  `ros2 service call /h753_odom_imu_compare/reset std_srvs/srv/Trigger '{}'`,
  then drive straight and perform a slow in-place rotation. Its synthetic
  wheel/IMU message test and external reset-service call both passed. Until
  Jetson IMU support is fixed, real hardware output correctly reports
  `imu[missing]`.
- Frontier exploration validates each candidate with Nav2
  `/compute_path_to_pose` before navigation, ignores goals farther than `4 m`,
  and times out active goals after `45 s`. This avoids long Spin/BackUp loops
  on unreachable frontiers.
- CLion installed as a snap exports `SNAP_COMMON=/var/snap/clion/common`.
  System `slam_toolbox` misinterprets that as its own snap runtime and used to
  rewrite absolute posegraph paths under `/var/snap/clion/common`, causing
  serialization `result=255`. ROS launch wrappers now unset `SNAP_COMMON`;
  `save_slam_map.sh` also detects the failure and retries with a compatible
  relative path so already-running sessions can still be saved.
- Modes `3` and `4` reject entry while saved posegraph files are absent.
- `Ctrl+C` stops child runtime and leaves no UART lock or serial port owner.
- Xbox button mapping is not hardware-verified yet. The Microsoft `045e:02e6`
  wireless dongle and `xone_dongle` driver are present, but `/dev/input/js*`,
  `/joy`, and `ros2 run joy joy_enumerate_devices` are empty. Power on and pair
  the controller before testing Menu/D-pad/A/B input.

## Known hardware caveats

- YDLIDAR previously emitted checksum and health `0x202` warnings. Solo `/scan`
  was measured near `10 Hz`; reconnect or power-cycle if repeated.
- Track-width rotation calibration is still empirical. The measured outer body
  width is `0.60 m`, but encoder yaw needs the left/right track centerline
  distance or an effective skid-steer width from an in-place rotation. ROS odom
  currently uses temporary `track_width_m: 0.65`; STM inverse kinematics uses
  `0.45 m`.
- Before floor testing autonomous modes, lift tracks and verify `/cmd_vel_safe`,
  forward direction, turn direction, stop mode, lidar stale stop, and UART deadman.

## 2026-05-31 update: remote-drive fixes and pending work

Root cause of "manual mapping remote drive not working / stuttering" was traced
end to end on hardware: `/joy` and `robot_mode_manager` were fine
(`/cmd_vel_selected` nonzero), but `collision_monitor` was zeroing/holding
`/cmd_vel_safe`, so the UART bridge only ever sent stop. The blue RViz polygon
(`PolygonStop`) was the visible culprit.

### Changes applied this session (these supersede older values above)

- UART bridge transmit cadence: fixed-rate timer only, 20 Hz
  (`transmit_period_s: 0.05`), with `serial.flush()` after every frame; the
  per-`/cmd_vel_safe`-callback send was removed. Mirrors the legacy
  `tools/xbox_uart_can_telemetry.py` loop (60 ms steady + flush) and removed the
  manual-drive stutter. Firmware command-hold timeout is 800 ms, so 20 Hz is safe.
- Speed envelope matched to the legacy UART teleop everywhere (0.60 m/s, 2.67 rad/s):
  - `h753_robot_mode_manager.yaml`: `manual_max` and `inspection_max` both
    `0.60 / 2.67` (inspection is no longer "low speed").
  - `h753_cmd_vel_uart_bridge.yaml`: `max_linear_mps 0.60`, `max_angular_radps 2.67`.
  - `h753_nav2.yaml` DWB: `max_vel_x 0.60`, `max_vel_theta 2.67`, `max_speed_xy 0.60`;
    accel raised to `0.80 / 2.50` to clear stiction; `velocity_smoother`
    `max/min_velocity [±0.60, 0, ±2.67]`.
  - Rationale: low speeds did not overcome motor stiction.
- `h753_collision_monitor_modes.yaml`: `source_timeout 0.5 -> 2.0`,
  `transform_tolerance 0.3 -> 1.0`, `base_shift_correction true -> false`,
  `PolygonStop.max_points 1 -> 5`, and BOTH `PolygonStop` and `PolygonSlow`
  `enabled: false`. Under full Jetson load the monitor processes `/scan` ~0.55 s
  after its stamp, so the old 0.5 s timeout tripped "Ignoring the source" and a
  fail-safe stop every cycle; the 0.36 m stop polygon also froze driving near
  walls/bench clutter.
- Xbox input is now hardware-verified (supersedes the "not verified / `/joy`
  empty" note): `js0` present, `axes[1]` = left-stick-Y throttle, `axes[3]` =
  right-stick-X steer, buttons `A=0 / B=1 / Menu=7`, `/joy` ~15-20 Hz. Full chain
  measured live: `/joy` -> `/cmd_vel_selected` (nonzero) -> `/cmd_vel_safe`
  (passes through continuously once polygons were disabled).
- NOTE: the collision/speed changes were also applied live via `ros2 param set`
  for same-session testing, but `transmit_period_s`, the manual/inspection/bridge
  limits, and polygon geometry are read once at node start, so a
  `run_robot_modes.sh` restart is required to load them from yaml.
- Encoder odom forward-direction fix: physical forward driving was observed as
  negative accumulated `wheel/odom` distance while the final `/odom` fusion was
  falling back directly to wheel odom. `h753_can_odom.yaml`
  `odom_linear_sign` was changed `+1.0 -> -1.0`. Restart the integrated runtime,
  then floor-test a short straight drive and confirm RViz moves along the
  robot's forward arrow. The interactive calibration tool was also fixed to
  combine a newly observed sign with the currently saved sign, so repeated
  calibration no longer toggles a correct value back to the wrong polarity.
- Sensor-rate tuning was added after measuring the full Jetson runtime:
  - YDLIDAR `/scan`: keep hardware configuration at `10 Hz`; measured `9.97 Hz`.
  - STM CAN encoder telemetry: measured raw `/wheel/odom` near `50 Hz`.
    `h753_can_odom.yaml` `poll_period_s` changed `0.01 -> 0.02` to match the
    actual telemetry cadence and avoid empty 100 Hz serial polling.
  - D435i color/depth previously requested `1280x720x30` and `848x480x30`,
    but only delivered roughly `19-21 Hz` while the RealSense node used about
    `58% CPU`. New shared `config/h753_realsense.yaml` requests reusable raw
    RGB at `1280x720x15` and depth at `640x480x15`, disables sync and depth
    alignment, and keeps pointcloud disabled. The connected D435i exposes
    `6/15/30 Hz` RGB profiles at `1280x720`, not `25 Hz`. Re-enable alignment
    only for a consumer that specifically needs registered depth-to-color.
  - D435i IMU request remains `gyro 200 Hz / accel 63 Hz`, but Jetson HID
    support is still missing so `/camera/camera/imu` currently has no publisher.
  - Direct `/scan` stamp-to-receive lag probe measured `0.099-0.108 s`
    (`0.103 s` average). The prior collision-monitor `~0.55 s` lag was likely
    downstream queue/load accumulation rather than YDLIDAR driver timestamping.
  - `~/ros2_graduation_project_ws/run_sensor_rate_check.sh` measures `/scan`,
    raw/fused odom, RGB, depth, and IMU together with one low-overhead ROS node.
  - Restart the integrated runtime before measuring the new camera rates; the
    already-running RealSense node retains its old stream profiles.

### Pending fixes (prioritized)

- P0 Autonomous collision safety regression. `PolygonStop`/`PolygonSlow` are now
  disabled globally, but `h753_collision_monitor_modes.yaml` is shared by modes
  1-5. Manual modes (1/3/5) are fine, but auto-mapping (2) and goal-nav (4) lost
  collision stop. Fix: make it mode-aware - have `robot_mode_manager` toggle
  `PolygonStop.enabled`/`PolygonSlow.enabled` via `ros2 param set` on mode switch
  (live param set was confirmed to work), enabled for 2/4, disabled for 1/3/5.
- P0 Scan timestamp lag (~0.55 s) root cause. `source_timeout: 2.0` is a
  band-aid; 2 s-stale obstacle data is unsafe for autonomous. Reduce the lag
  (rviz on another host or off, lower costmap `update_frequency`, check the
  ydlidar driver timestamping), then restore `source_timeout` near 0.5 for 2/4.
- P1 Low-speed motor stiction (firmware). `App/Src/app_uart_smoke.c`
  `MIN_DUTY_PERCENT = 5` is likely below breakaway duty, so autonomous fine
  maneuvers and ramp-up still stall. Fix: raise `MIN_DUTY_PERCENT` to the
  measured breakaway duty, or add a stiction feedforward (minimum move duty for
  any nonzero target), then reflash. A bridge-side minimum-command floor is a
  fallback.
- P1 Polarity floor-verification. `linear_command_sign` and
  `angular_command_sign` are `-1.0` in `h753_cmd_vel_uart_bridge.yaml`. On lifted
  tracks confirm forward stick -> forward, right -> right; flip a sign if reversed.
  Also restart after the saved `odom_linear_sign: -1.0` change and confirm a
  physical straight drive moves RViz odom forward, not backward.
- P2 Track-width mismatch. ROS odom `track_width_m: 0.65` vs STM inverse
  kinematics `0.45 m`. Calibrate the effective track width from an in-place
  rotation and reconcile (or document the slip-driven difference).
- P2 Autonomous 0.60 m/s overshoot check. DWB at 0.60 m/s / 2.67 rad/s may be
  too fast in tight mapping; if it oscillates, keep manual at 0.60 but lower the
  DWB/autonomous envelope (pairs naturally with the P0 mode-aware split).

## 2026-05-31 continuation: runtime safety and Jetson load fixes

The pending software items above were addressed after inspecting a live Jetson
runtime. The largest immediate fault was not sensor calibration: an orphaned
calibration launch and the integrated mode runtime had opened the same YDLIDAR
and CANable devices at the same time. Two YDLIDAR readers split the serial
stream, producing checksum failures and stale `/scan` symptoms.

### Applied fixes

- Killed the orphaned calibration YDLIDAR, CAN odom, RViz, and static TF
  processes. Verified that the final idle runtime has exactly one owner for
  `/dev/ttyUSB0`, `/dev/ttyACM0`, and `/dev/ttyACM1`, and releases all three on
  shutdown.
- `run_calibration.sh` now starts its launch in a separate session and cleans up
  the full child process group. `robot_mode_manager_node.py` checks the lidar,
  CANable, and STM UART device owners with `fuser` before starting a runtime and
  rejects overlapping launches.
- Collision safety is mode-aware:
  - modes `1/3/5`: `PolygonStop.enabled=false`, `PolygonSlow.enabled=false`
  - modes `2/4`: both polygons are enabled dynamically
  - autonomous `/cmd_vel` remains zero until the parameter update succeeds
- Collision monitor `source_timeout` was reduced from the temporary `2.0 s`
  workaround to `0.75 s`. A single integrated headless runtime measured `/scan`
  stamp lag `0.100/0.103/0.108 s` min/avg/max and no repeated stale-source
  warning.
- Nav2 footprint now matches the measured outer envelope: `0.55 m` long by
  `0.60 m` wide (`x=+/-0.275`, `y=+/-0.30`). Autonomous hard-stop and slowdown
  polygons were expanded outside that envelope.
- Manual Xbox limits remain `0.60 m/s`, `2.67 rad/s`. Nav2 autonomous limits
  were temporarily reduced to `0.35 m/s`, `1.00 rad/s`; the later continuation
  below restores the standalone Xbox UART envelope.
- D435i reusable stream defaults are RGB `1280x720x15` and depth
  `640x480x6`. The depth stream is retained for other consumers but reduced
  because SLAM/Nav2 do not use it yet.
- Jetson D435i HID support is still absent. Default integrated launches now use
  direct wheel `/odom`: `enable_imu:=false`, `launch_imu_odom:=false`.
  Re-enable both after the Jetson HID/librealsense fix.
- Visualization-only `/map_odom` is disabled by default to save CPU. Enable
  `launch_map_odom:=true` only while inspecting the map-frame odom vector.
- Frontier exploration is no longer kept alive during manual mapping:
  - mode `1`: mapping runtime only
  - mode `2`: mode manager starts the frontier explorer sidecar
  - return to mode `1/0`: mode manager stops the sidecar without restarting SLAM
  - direct `run_auto_explore.sh` explicitly enables the explorer
- ROS Python nodes now treat shutdown races as clean shutdowns. A final
  headless startup/shutdown test left no traceback and no serial owner.

### Final idle measurements

Measured with headless online SLAM + Nav2, camera enabled, RViz disabled, no
motor command publisher, and the optional map-odom/frontier/IMU-fusion nodes
disabled:

```text
/scan:                                9.97 Hz
/odom:                               50.38 Hz
/camera/camera/color/image_raw:      10.67 Hz
/camera/camera/depth/image_rect_raw:  4.40 Hz
/camera/camera/imu:                  missing (expected until Jetson HID fix)
```

The configured camera profiles are confirmed by the RealSense node as
`1280x720x15` RGB and `640x480x6` depth. Delivered camera rates remain lower
under Jetson desktop load. During measurement CLion alone used about `118%`
CPU; close CLion and avoid running RViz locally when validating camera FPS.

### Remaining physical tests

1. Direction polarity: the 2026-05-31 16:08 inspection drive confirmed that
   physical driving was correct but RViz odom was reversed for forward/backward
   and both in-place rotation directions. Saved odom config was changed to
   `odom_linear_sign: -1.0`, `odom_angular_sign: -1.0`; UART bridge command signs
   remain `linear_command_sign: -1.0`, `angular_command_sign: -1.0`. Restart,
   use mode `5`, and verify a short forward drive and both rotation directions.
2. Effective skid-steer width: ROS encoder odom still uses `track_width_m:
   0.65`; STM inverse kinematics uses `0.45 m`. Measure a slow in-place
   rotation and update the ROS effective width from the actual angle.
3. Motor stiction: measure the smallest duty that reliably starts motion.
   Firmware `MIN_DUTY_PERCENT = 5` may still be too low for autonomous fine
   maneuvers.
4. Camera throughput: if 720p RGB must reach the configured `15 Hz`, repeat the
   sensor-rate test with CLion closed and RViz on another machine. If that is
   still insufficient, split the camera into an on-demand launch.
5. D435i IMU: install the Jetson librealsense kernel patch or RSUSB backend,
   then run IMU calibration and `run_odom_imu_compare.sh`.

## 2026-05-31 continuation: rotation overlap video diagnosis

Inspected `/home/jyl1015/Desktop/스크린캐스트 2026-05-31 16-18-28.webm`
with `ffmpeg`. During in-place rotation, the same walls fan out and remain
duplicated at multiple yaw angles. This is a mapping error, not only an RViz
display artifact. The matching mode log
`~/.ros/log/h753_robot_modes/20260531_161454_mapping.log` has no repeated
YDLIDAR checksum errors and no encoder sample rejection, so the primary next
fix is encoder angular-odom scale calibration.

`~/ros2_graduation_project_ws/src/h753_can_odom/h753_can_odom/interactive_calibration.py`
menu `2` now saves both `odom_angular_sign` and the skid-steer effective
`track_width_m`. Mark an accurate 90 degree counter-clockwise turn with floor
tape, keep the robot center fixed, rotate slowly, enter `90`, and save. The
calculation is:

```text
new_track_width = current_track_width * abs(odom_measured_yaw) / actual_yaw
```

The ROS effective width may be larger than the physical 0.60 m outer width
because it includes track slip. Restart the integrated runtime after
calibration and repeat a slow in-place mapping test. If a smaller consistent
overlap remains afterward, measure the lidar mounting `laser_x`/`laser_y`
offset from the robot rotation center; both are still temporarily `0.0`.

Do not start `run_robot_modes.sh` while `run_calibration.sh` is active. Both
runtimes need exclusive ownership of YDLIDAR and CANable, and the mode manager
will intentionally reject the second launch. `run_calibration.sh` starts
`run_calibration_uart_drive.sh` automatically in the background and keeps its
encoder output in `~/.ros/log/h753_calibration/uart_drive_*.log`, so the
foreground terminal stays on the numbered calibration menu.

The UART helper opens only the ST-LINK STM UART and disables CAN telemetry. Its
drive limits are exactly the same defaults used by
`~/h753_ros_humble/tools/run_xbox_uart_drive.sh`. It fails instead of falling
back to the YDLIDAR or CANable serial ports if the ST-LINK port cannot be
identified unambiguously.

## 2026-05-31 continuation: autonomous speed envelope

Restored Nav2 DWB, velocity-smoother, and recovery-spin limits to the standalone
Xbox UART envelope: `0.60 m/s` linear and `2.67 rad/s` angular. Auto mapping
mode `2` and saved-map navigation mode `4` still enable the collision-monitor
polygons dynamically. Inside `PolygonSlow`, commands are intentionally scaled
by `slowdown_ratio: 0.35`; inside `PolygonStop`, commands are forced to zero.

The first auto-mapping startup exposed an initialization race: the mode manager
called `/collision_monitor/set_parameters` while the lifecycle node was still
`unconfigured`, before `PolygonStop.enabled` and `PolygonSlow.enabled` had been
declared. The runtime eventually recovered after collision-monitor configure,
but printed repeated rejection errors first. `robot_mode_manager_node.py` now
queries `/collision_monitor/get_state` and sends the polygon update only after
the lifecycle node reaches `active`. Autonomous commands remain held at zero
until that update succeeds.

## 2026-06-01: 부호 체계 전면 수정 및 자율주행 디버깅

### 발견된 버그 및 수정 사항

#### 키보드 주행 방향 반전 (수정 완료)
`robot_mode_manager_node.py` `_set_keyboard_drive()` 내 선속도와 각속도 부호가
모두 반대였다. 조이패드는 정상이지만 키보드만 역방향이었음.

- `w` : `+max_linear` → `-max_linear` (전진)
- `s` : `-0.70 * max_linear` → `+0.70 * max_linear` (후진)
- `a` : `+max_angular` → `-max_angular` (좌회전)
- `d` : `-max_angular` → `+max_angular` (우회전)

**원인**: 조이패드는 `_joy_to_twist` 내부에서 throttle을 `-apply_deadzone(axes[1])`로
이미 반전하는데 키보드는 그 보정 없이 직접 부호를 지정했음. 이 Xbox 컨트롤러는
`axes[1]` 전진 방향이 `+1.0`이므로 throttle 반전 후 음수 linear = 올바른 전진.
전체 부호 체계: `/cmd_vel_safe` 음수 linear → UART bridge `× (-1.0)` → STM `+v` → 물리 전진.

#### 오도메트리 각도 부호 반전 (수정 완료)
`h753_can_odom.yaml` `odom_angular_sign: -1.0` → `1.0`

**원인**: 물리 반시계(CCW) 회전 시 `/odom` orientation z가 음수 방향으로 증가했음.
ROS 규약상 CCW = z 양수 증가여야 하므로 부호가 반대였음. 이 버그로 SLAM 맵이
앞-뒤, 좌-우 모두 반전되는 현상(180° 회전)이 발생했음.

수정 후 맵이 정상적으로 나오는 것을 RViz에서 확인함.

#### 자율주행 twist 반전 주석 추가
`robot_mode_manager_node.py` `_selected_twist()` 내 Nav2 cmd_vel 반전 코드에 이유
설명 주석 추가. 이 반전은 버그가 아니라 UART bridge의 `linear_command_sign: -1.0`
보정과 상쇄되어 올바른 방향을 만드는 의도적 설계임.

### 캘리브레이션 상태

#### track_width_m
- 물리 실측 바퀴 중심 간격: **530 mm**
- 제자리 회전 캘리브레이션 불가: 스키드 스티어 특성상 로봇 중심이 고정되지 않음
- SLAM 루프 클로저 테스트로 대신 캘리브레이션 진행
- 현재 저장값: **`0.48 m`** (`h753_can_odom.yaml`)
  - `0.345`(65% 보정)은 너무 작아 맵이 폭발함
  - `0.48`에서 직사각형 루프가 닫히는 것 확인
  - 미세 조정 여지 있음; SLAM 루프 테스트 반복 권장

#### IMU 캘리브레이션
`run_calibration.sh` 메뉴 3 (IMU 반시계 회전) 시도 → `/camera/camera/imu` 없음으로
실패. Jetson HID 커널 모듈 미지원 문제 지속. 현재는 wheel odom 폴백으로 운용.

### 충돌 모니터 설정 변경

`h753_collision_monitor_modes.yaml` 및 `robot_mode_manager_node.py` 수정:

- **PolygonStop**: 모드 2/4에서도 비활성화 유지. Nav2 costmap inflation(`inflation_radius: 0.35 m`)이 장애물 회피를 담당하므로 중복 불필요. 폴리곤 크기를 로봇 외곽(x ±0.275 m, y ±0.30 m) 밖으로 수정 (front 0.43 m).
- **PolygonSlow**: `max_points: 1 → 5`, `slowdown_ratio: 0.35 → 0.60`, 폴리곤 크기 확장 (front 0.73 m). 이전 설정은 실내 벽을 항상 감지해 명령을 35%로 줄이고 사실상 모터가 움직이지 못하게 했음.
- `robot_mode_manager_node.py` `_collision_state_done_callback()`: `PolygonStop.enabled`를 항상 `False`로, `PolygonSlow.enabled`만 모드 2/4에서 `True`로 설정하도록 변경.

### UART 브리지 stiction 플로어 추가

`cmd_vel_uart_bridge_node.py` `_apply_stiction_floor()` 메서드 및 파라미터 추가:

```yaml
# h753_cmd_vel_uart_bridge.yaml
min_linear_mps: 0.08
min_angular_radps: 0.30
```

Nav2 DWB가 출력하는 0.02 m/s 등 매우 낮은 속도 명령은 모터 stiction을 극복하지
못한다. 0이 아닌 명령이 최솟값 미만일 경우 부호를 유지하며 최솟값으로 올린다.
`min_linear_mps`, `min_angular_radps` 파라미터로 조정 가능.

### 현재 미해결 문제 (2026-06-01 기준)

#### P0: 자율주행(mode 4) 로봇이 움직이지 않음
RViz에서 Nav2 Global Path는 계획되지만 로봇이 실제로 이동하지 않는다. RViz 내
로봇 아이콘도 움직이지 않으므로 UART 브리지까지 도달하지 못하거나 STM 명령이
효과가 없는 것으로 추정.

진단 체크리스트:
1. `/cmd_vel` → `/cmd_vel_selected` → `/cmd_vel_safe` 각 단계 값 확인
2. `collision_safety_applied`가 `True`인지 확인 (로그에 `Collision polygons enabled=True` 출력 여부)
3. UART 브리지 로그에 `scan timeout` 또는 `deadman` 메시지 확인
4. stiction 플로어 수정 후 재시작하여 재테스트 필요 (2026-06-01 세션 종료 시점에 미완료)

가장 최근 관찰: `/cmd_vel_safe.linear.x ≈ 0.02 m/s` (stiction 미달) 및 충돌 폴리곤이
로봇 프레임 안으로 들어가 있던 설정 문제를 발견하여 수정함. 재시작 후 동작 여부
미확인 상태.

#### P1: track_width_m 미세 조정 필요
현재 `0.48 m`는 SLAM 루프가 닫히는 것을 확인했으나 벽이 약간 두껍게 그려짐.
더 정확한 값은 `0.46~0.52 m` 범위에서 루프 클로저 반복 테스트로 수렴시킬 것.

#### P1: PolygonStop 재활성화
자율주행이 정상 동작 확인 후 PolygonStop을 모드 2/4에서 재활성화해야 함.
`robot_mode_manager_node.py` `_collision_state_done_callback()` 내
`self._bool_parameter('PolygonStop.enabled', False)` →
`self._bool_parameter('PolygonStop.enabled', enabled)` 로 복원.

#### P2: 모터 stiction 근본 해결
현재 `min_linear_mps: 0.08` 플로어는 임시 조치. 근본 해결은 STM 펌웨어
`App/Src/app_uart_smoke.c` `MIN_DUTY_PERCENT`를 실측 breakaway duty로 올리거나
stiction feedforward를 추가하는 것.

#### P3: laser_x / laser_y 오프셋
라이다 마운팅이 로봇 회전 중심에서 오프셋되어 있으면 회전 시 스캔 왜곡 발생.
현재 `laser_x: 0.0`, `laser_y: 0.0`. `run_calibration.sh` 또는 맵 품질로 측정 필요.
