#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/empty.hpp> // 新增消息类型

class ConfirmedController : public rclcpp::Node {
public:
    ConfirmedController() : Node("experiment_control") {
        // 订阅位姿（仅规划）
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_arm_pose", 10, std::bind(&ConfirmedController::pose_callback, this, std::placeholders::_1));
        
        // 订阅确认信号（触发执行）
        exec_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "execute_arm_plan", 10, std::bind(&ConfirmedController::execute_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "控制节点已启动。流程：UI计算 -> C++规划 -> 用户点击UI执行。");
    }

    void init_move_group() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "rm_group");
    }

private:
    // 规划回调：只计算，不运动
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到目标，开始尝试规划...");
        move_group_->setPoseTarget(*msg);

        // 进行规划并存储结果
        bool success = (move_group_->plan(last_plan_) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            plan_available_ = true;
            RCLCPP_INFO(this->get_logger(), "规划成功！请在 UI 上点击“执行”确认。");
        } else {
            plan_available_ = false;
            RCLCPP_ERROR(this->get_logger(), "规划失败，无法执行。");
        }
    }

    // 执行回调：用户确认后真正运动
    void execute_callback(const std_msgs::msg::Empty::SharedPtr) {
        if (plan_available_) {
            RCLCPP_INFO(this->get_logger(), "接收到确认指令，机械臂开始运动！");
            move_group_->execute(last_plan_);
            plan_available_ = false; // 执行后清除状态
        } else {
            RCLCPP_WARN(this->get_logger(), "当前没有有效的规划结果，请先进行规划。");
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan last_plan_; // 缓存规划结果
    bool plan_available_ = false;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr exec_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConfirmedController>();
    node->init_move_group();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}