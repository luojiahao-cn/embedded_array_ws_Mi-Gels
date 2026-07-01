# Tools

Small shell wrappers for repeatable local workflows. These scripts should stay
thin: package-specific ROS nodes belong under the package's `scripts/`
directory.

## Manual MagGrad Recording

Terminal 1 launches the manual recorder from this workspace:

```bash
./tools/run_stm32_manual_migels.sh
```

To activate the ROS environment in a shell without launching anything:

```bash
source ./tools/source_migels_ros_env.sh
```

When controlling zlab over SSH but showing ROS GUI windows on zlab's physical
display:

```bash
source ./tools/zlab_ros_env.sh
```

Terminal 2 controls start/stop with Enter:

```bash
./tools/manual_record_enter.sh
```

Both scripts support `--help`.

## Scripts

| Script | Purpose |
| --- | --- |
| `source_migels_ros_env.sh` | Sources ROS, `zlab_robots`, and this workspace; verifies every package in this worktree resolves here |
| `zlab_ros_env.sh` | Sets zlab's physical display variables, then sources `source_migels_ros_env.sh` |
| `run_stm32_manual_migels.sh` | Sources `source_migels_ros_env.sh`; launches `stm32_manual.launch` |
| `manual_record_enter.sh` | Publishes `std_msgs/Bool` start/stop triggers to `/maggrad_manual_record/record_trigger` |

## Environment

| Variable | Default | Used by |
| --- | --- | --- |
| `ROS_DISTRO` | `noetic` | Environment and launch scripts |
| `ZLAB_ROBOTS_WS` | `$HOME/zlab_robots` | Environment and launch scripts |
| `MIGELS_RUNTIME_CONFIG` | `<workspace>/migels_runtime.yaml` | Environment and launch scripts |
| `TOPIC` | `/maggrad_manual_record/record_trigger` | `manual_record_enter.sh` |
