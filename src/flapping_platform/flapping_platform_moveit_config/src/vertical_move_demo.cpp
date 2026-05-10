#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * @brief 使用 MoveIt2 C++ API 控制机械臂末端做笛卡尔直线往复运动
 * 
 * 运行前提：
 *   1. 先启动 flapping_platform_gz_moveit.sh（启动 Gazebo + MoveIt2 move_group）
 *   2. 再运行本节点
 * 
 * 功能：
 *   1. 通过 ROS2 参数指定起始位置 (start_x, start_y, start_z)、目标 Z 高度 (target_z) 和往复次数 (num_cycles)
 *   2. 先通过关节空间运动到起始位姿
 *   3. 然后循环执行往复运动：每次去程和回程前实时计算笛卡尔直线轨迹并执行
 *   4. 直线运动过程中保持 x, y 和姿态不变
 *   5. 每次 execute 后通过 TF 等待末端到达目标 Z 位置
 * 
 * 关于 "什么位置直线距离更长"：
 *   RM63 机械臂的大臂 0.38m + 小臂 0.405m + 腕部 0.132m
 *   在 x=0.3~0.5m, y=0（正前方）的区域内，Z 轴直线行程最大，可达约 0.5~0.6m
 *   太靠近基座 (x<0.2) 或太远 (x>0.7) 时，关节限位会限制垂直行程
 *   推荐参数：start_x=0.4 start_z=0.3 target_z=0.9 可获得最长行程 ~0.6m
 * 
 * 使用示例：
 *   # 在 (-0.6, 0, 0.3) 和 (-0.6, 0, 0.6) 之间往复 3 次
 *   ros2 run flapping_platform_moveit_config vertical_move_demo \
 *     --ros-args -p start_x:=-0.6 -p start_y:=0.0 -p start_z:=0.3 -p target_z:=0.6 -p num_cycles:=3
 * 
 *   # 在 (0.4, 0, 0.3) 和 (0.4, 0, 0.9) 之间往复 5 次（最长行程）
 *   ros2 run flapping_platform_moveit_config vertical_move_demo \
 *     --ros-args -p start_x:=0.4 -p start_y:=0.0 -p start_z:=0.3 -p target_z:=0.9 -p num_cycles:=5
 */

// 等待末端到达目标 Z 值（通过 TF 获取 arm1_link6 在 arm1_base_link 下的位姿）
void wait_for_z_reached(double target_z, double tolerance, double timeout_sec,
                         rclcpp::Node::SharedPtr node,
                         tf2_ros::Buffer& tf_buffer)
{
  auto start_time = node->now();
  while (rclcpp::ok())
  {
    // 使用 TF 查询 arm1_link6 在 arm1_base_link 坐标系下的位置，与目标坐标系一致
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer.lookupTransform(
          "arm1_base_link", "arm1_link6",
          tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node->get_logger(), "TF 查询失败: %s", ex.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    double current_z = transform.transform.translation.z;
    RCLCPP_INFO(node->get_logger(), "等待末端到达目标 Z=%.3f, 当前 Z=%.3f (容差 %.3f)",
                target_z, current_z, tolerance);
    if (std::abs(current_z - target_z) < tolerance)
    {
      RCLCPP_INFO(node->get_logger(), "末端已到达目标 Z=%.3f", target_z);
      return;
    }
    double elapsed = (node->now() - start_time).seconds();
    if (elapsed > timeout_sec)
    {
      RCLCPP_WARN(node->get_logger(), "等待末端到达目标 Z=%.3f 超时 (%.1f s)", target_z, elapsed);
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

// 计算笛卡尔直线轨迹并执行
bool plan_and_execute_cartesian(moveit::planning_interface::MoveGroupInterface& mg,
                                 const geometry_msgs::msg::Pose& target_pose,
                                 const std::string& label,
                                 rclcpp::Node::SharedPtr node,
                                 tf2_ros::Buffer& tf_buffer)
{
  // 获取当前位姿（默认在 planning frame = world），然后转换到 arm1_base_link
  geometry_msgs::msg::PoseStamped ps = mg.getCurrentPose("arm1_link6");
  geometry_msgs::msg::Pose start_pose;
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer.lookupTransform(
        "arm1_base_link", ps.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(ps.pose, start_pose, tf);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node->get_logger(), "TF 变换失败: %s，使用原始位姿", ex.what());
    start_pose = ps.pose;
  }
  // 修正：start_pose 的 orientation 可能是非单位四元数，强制重设为期望方向
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 1.0;

  RCLCPP_INFO(node->get_logger(), "计算轨迹: z=%.3f → z=%.3f", start_pose.position.z, target_pose.position.z);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = mg.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, true);

  if (fraction < 0.9)
  {
    RCLCPP_ERROR(node->get_logger(), "%s 轨迹规划失败 (完成度 %.1f%%)", label.c_str(), fraction * 100.0);
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "%s 轨迹规划完成度: %.1f%%，执行中...", label.c_str(), fraction * 100.0);

  for (int attempt = 0; attempt < 3; ++attempt)
  {
    auto result = mg.execute(trajectory);
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node->get_logger(), "%s 执行成功", label.c_str());
      return true;
    }
    RCLCPP_WARN(node->get_logger(), "%s 执行失败 (attempt %d)，重试...", label.c_str(), attempt + 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  RCLCPP_ERROR(node->get_logger(), "%s 执行失败，放弃", label.c_str());
  return false;
}

int main(int argc, char *argv[])
{
  // 1. 初始化 ROS2，启用仿真时间以解决 getCurrentPose 返回全零的问题
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("use_sim_time", true);
  auto node = rclcpp::Node::make_shared("vertical_move_demo_node", node_options);

  // 声明并获取 ROS2 参数
  // 起始位置（机械臂先运动到这个点）
  node->declare_parameter<double>("start_x", -0.4);
  node->declare_parameter<double>("start_y", 0.0);
  node->declare_parameter<double>("start_z", 0.3);
  // 目标 Z 高度（直线运动的目的地）
  node->declare_parameter<double>("target_z", 0.7);
  // 往复次数
  node->declare_parameter<int>("num_cycles", 5);

  double start_x = node->get_parameter("start_x").as_double();
  double start_y = node->get_parameter("start_y").as_double();
  double start_z = node->get_parameter("start_z").as_double();
  double target_z = node->get_parameter("target_z").as_double();
  int num_cycles = node->get_parameter("num_cycles").as_int();

  RCLCPP_INFO(node->get_logger(), "=== 配置参数 ===");
  RCLCPP_INFO(node->get_logger(), "起始位置: (%.3f, %.3f, %.3f)",
              start_x, start_y, start_z);
  RCLCPP_INFO(node->get_logger(), "目标 Z  :  %.3f", target_z);
  RCLCPP_INFO(node->get_logger(), "运动方向: %s",
              (target_z > start_z) ? "先向上 ↑ 再向下 ↓" : "先向下 ↓ 再向上 ↑");
  RCLCPP_INFO(node->get_logger(), "单程行程: %.3f m", std::abs(target_z - start_z));
  RCLCPP_INFO(node->get_logger(), "往复次数: %d 次", num_cycles);

  // 2. 创建异步执行器，MoveIt 需要异步 spin
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // 3. 创建 TF 监听器（用于获取末端在 arm1_base_link 坐标系下的位姿）
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // 4. 创建 MoveGroup 接口，连接到规划组 "arm1"
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm1");

  // 5. 设置参考坐标系为基坐标系
  move_group.setPoseReferenceFrame("arm1_base_link");

  // 6. 设置规划参数
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.allowReplanning(true);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.05);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // ============================================================
  // 第一步：先通过关节空间运动到起始位姿
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "\n=== 第一步：关节空间运动到起始位姿 ===");

  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = start_x;
  start_pose.position.y = start_y;
  start_pose.position.z = start_z;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 1.0;

  move_group.setPoseTarget(start_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan_to_start;
  bool plan_success = (move_group.plan(plan_to_start) == moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_success)
  {
    RCLCPP_INFO(node->get_logger(), "规划成功，执行到起始位姿...");
    move_group.execute(plan_to_start);
    RCLCPP_INFO(node->get_logger(), "到达起始位姿！");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "无法规划到起始位姿 (%.3f, %.3f, %.3f)，退出！",
                 start_x, start_y, start_z);
    RCLCPP_ERROR(node->get_logger(), "可能是该位置在工作空间之外或存在碰撞");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // ============================================================
  // 第二步：终点位姿（只有 z 不同）
  // ============================================================
  geometry_msgs::msg::Pose end_pose = start_pose;
  end_pose.position.z = target_z;

  // ============================================================
  // 第三步：循环执行往复运动（每次实时计算轨迹）
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "\n=== 第三步：开始笛卡尔直线往复运动 ===");
  RCLCPP_INFO(node->get_logger(), "z: %.3f → %.3f → %.3f ... 共 %d 次往复",
              start_z, target_z, start_z, num_cycles);

  for (int cycle = 0; cycle < num_cycles; ++cycle)
  {
    RCLCPP_INFO(node->get_logger(), "\n--- 第 %d / %d 次往复 ---", cycle + 1, num_cycles);

    // 去程：实时计算并执行
    if (!plan_and_execute_cartesian(move_group, end_pose, "去程 ↑", node, tf_buffer))
    {
      RCLCPP_ERROR(node->get_logger(), "去程失败，退出循环");
      break;
    }
    RCLCPP_INFO(node->get_logger(), "去程执行完毕，等待末端到达目标 Z...");
    wait_for_z_reached(target_z, 0.02, 10.0, node, tf_buffer);

    // 回程：实时计算并执行
    if (!plan_and_execute_cartesian(move_group, start_pose, "回程 ↓", node, tf_buffer))
    {
      RCLCPP_ERROR(node->get_logger(), "回程失败，退出循环");
      break;
    }
    RCLCPP_INFO(node->get_logger(), "回程执行完毕，等待末端回到起点 Z...");
    wait_for_z_reached(start_z, 0.02, 10.0, node, tf_buffer);
  }

  RCLCPP_INFO(node->get_logger(), "\n=== 所有 %d 次往复运动完成！===", num_cycles);

  // 清理
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
