# 多 Policy Profile 遥控切换说明

本文档说明障碍赛部署时如何在同一个 `deploy_node` 中通过遥控器切换多个 HIMLoco policy。核心约定是：每个障碍 policy 对应一个完整机器人 YAML，顶层 profile 清单只负责把遥控器输入映射到这些 YAML。

## 相关文件

- profile 清单示例：`deploy_cpp/config/policy_profiles/obstacle_course.yaml`
- 单个 policy 的完整 YAML 示例：
  - `deploy_cpp/config/robots/mybot_v2_real.yaml`
  - `deploy_cpp/config/robots/mybot_stair.yaml`
  - `deploy_cpp/config/robots/mybot_crawl.yaml`
  - `deploy_cpp/config/robots/mybot_foot_adduction.yaml`
- 主要实现：
  - `deploy_cpp/src/policy_profile_manager.cpp`
  - `deploy_cpp/src/deploy_node.cpp`

## Profile 清单格式

```yaml
initial_profile: walk

profiles:
  - id: walk
    name: normal_walk
    config_file: ../robots/mybot_v2_real.yaml
    joy_axis:
      index: 7
      value: -32767

  - id: stair
    name: stair
    config_file: ../robots/mybot_stair.yaml
    joy_axis:
      index: 6
      value: 32767

  - id: crawl
    name: crawl
    config_file: ../robots/mybot_crawl.yaml
    joy_axis:
      index: 7
      value: 32767

  - id: foot_adduction
    name: foot_adduction
    config_file: ../robots/mybot_foot_adduction.yaml
    joy_axis:
      index: 6
      value: -32767
```

字段含义：

- `initial_profile`：`deploy_node` 启动后默认激活的 profile id。
- `id`：稳定标识，会显示在日志和 CSV 的 `profile_id` 列中。
- `name`：便于阅读的人类名称。
- `config_file`：指向完整机器人 YAML 的路径，相对于 profile 清单文件解析。
- `joy_button`：可选，`sensor_msgs/msg/Joy.buttons[index]`，按下后选择该 profile。
- `joy_axis`：可选，使用 `sensor_msgs/msg/Joy.axes[index]` 的指定值选择该 profile。方向键可写原始值 `±32767`；如果运行时收到的是 `joy_node` 常见归一化值 `±1.0`，也会自动识别。

`joy_axis` 示例：

```yaml
joy_axis:
  index: 7
  value: -32767
```

`joy_button` 和 `joy_axis` 至少填写一个。二者都填写时，任意一个触发都会选择该 profile。

## 单个 Policy YAML 规则

每个 profile 指向的 YAML 都必须是一个完整的普通机器人配置。不同 policy 可以在各自 YAML 中独立配置这些字段：

- `policy_path`
- `default_dof_pos`
- `standup_target_pos`
- `policy_dof_pos`
- `kp_joint`、`kd_joint`
- `action_scale`
- `cmd_vx_min/max`、`cmd_vy_min/max`、`cmd_yaw_min/max`
- `cmd_deadband`、`standup_duration`

以下字段被视为硬件不变量。所有启用的 profile 必须保持一致，否则启动时会直接报错退出，避免运行中误换电机映射或机器人模型：

- `num_of_dofs`
- `joint_names`、`joint_controller_names`
- `joint_mapping`
- `motor_is_reversed`
- `joint_transmission_ratio`
- `port0`、`port1`
- `imu_topic`
- `urdf_relpath`、`mujoco_xml_relpath`、`isaac_xml_relpath`
- `control_dt`
- `joint_pos_lower`、`joint_pos_upper`

## 启动方式

单 policy 模式仍然兼容：

```bash
ros2 launch deploy_cpp deploy.launch.py \
  robot_config_file:=/path/to/deploy_cpp/config/robots/mybot_v2_real.yaml
```

多 policy 模式：

```bash
ros2 launch deploy_cpp deploy.launch.py \
  policy_profiles_file:=/path/to/deploy_cpp/config/policy_profiles/obstacle_course.yaml
```

MuJoCo 仿真模式也支持同一个 profile 清单：

```bash
ros2 launch deploy_cpp sim.launch.py \
  policy_profiles_file:=/path/to/deploy_cpp/config/policy_profiles/obstacle_course.yaml
```

## 运行时切换流程

按下某个 profile 对应的遥控器按钮或方向键轴后，节点会自动执行以下流程：

1. 立即清零遥控速度指令。
2. 如果当前正在 RL，则 flush 当前 RL 关节 CSV 片段。
3. 从当前实测关节姿态平滑插值到目标 profile 的 `policy_dof_pos`。
4. 插值过程使用目标 profile 的 `kp_joint`、`kd_joint` 和 `standup_duration`。
5. 切换到目标 profile 预加载好的 `PolicyRunner`。
6. 重置 policy history 和 last actions。
7. 自动进入 `RL` 状态。
8. 进入新 RL 后速度继续保持 0，直到摇杆回到死区一次，防止切换后突然冲出去。

急停按钮会打断正在进行的 profile 切换，并强制进入 `IDLE`。

## 新增一个障碍 Policy

1. 复制一个已经验证过的机器人 YAML。
2. 修改 `policy_path` 指向新的 JIT policy 文件。
3. 设置该 policy 对应的 `default_dof_pos`、`standup_target_pos` 和 `policy_dof_pos`。
4. 根据训练设置调整 `action_scale`、PD 增益和速度范围；如果新 policy 只需要复用已有控制参数，则只改模型路径和三组姿态。
5. 确认所有硬件不变量与其他启用 profile 完全一致。
6. 在 `obstacle_course.yaml` 中新增一个 profile，填写唯一的 `id` 和未占用的 `joy_button` 或 `joy_axis`。
7. 先用 `debug_no_motor:=true` 或 MuJoCo 验证切换流程，再上实机。

## 实机前检查清单

- 运行 `colcon build --packages-select deploy_cpp`。
- 先用 `debug_no_motor:=true` 启动，确认 profile 清单可以加载。
- 逐个按下 profile 按钮或方向键，确认日志显示 `SWITCH_PROFILE` 后进入 `RL`。
- 确认状态行中的 `profile=<id>` 与当前障碍策略一致。
- 确认 CSV 日志包含 `profile_id` 列。
- 上实机时先在 `IDLE` 下逐个切换 profile，确认目标姿态正确，再测试 RL 中切换。
