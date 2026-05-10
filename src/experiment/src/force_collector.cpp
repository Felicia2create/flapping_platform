#include <rclcpp/rclcpp.hpp>
#include <rm_ros_interfaces/msg/sixforce.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <filesystem>

namespace fs = std::filesystem;

class ForceCollector : public rclcpp::Node {
public:
    ForceCollector() : Node("force_collector") {
        //绝对路径
        this->declare_parameter("data_path", "/home/dushi/flapping_platform_ws/src/experiment/data");
        std::string folder_path = this->get_parameter("data_path").as_string();
        try {
            if (!fs::exists(folder_path)) {
                fs::create_directories(folder_path);
                RCLCPP_INFO(this->get_logger(), "创建了新目录: %s", folder_path.c_str());
            }
            if (!folder_path.empty() && folder_path.back() != '/') {
                folder_path += "/";
            }
        // 1. 创建文件名
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << folder_path << "force_data_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << ".csv";
        filename_ = ss.str();

        // 2. 初始化 CSV 文件头
        file_out_.open(filename_, std::ios::out);
        if (file_out_.is_open()) {
            file_out_ << "timestamp_ns,fx,fy,fz,tx,ty,tz" << std::endl;
            RCLCPP_INFO(this->get_logger(), "数据记录已启动，保存至: %s", filename_.c_str());
        }
        else {
                RCLCPP_ERROR(this->get_logger(), "无法打开文件进行写入: %s", filename_.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "路径处理出错: %s", e.what());
        }
        // 3. 订阅力传感器话题 (选择你认为更准的 zero_force)
        subscription_ = this->create_subscription<rm_ros_interfaces::msg::Sixforce>(
            "/rm_driver/udp_six_zero_force", 10,
            std::bind(&ForceCollector::force_callback, this, std::placeholders::_1));
    }

    ~ForceCollector() {
        if (file_out_.is_open()) {
            file_out_.close();
            RCLCPP_INFO(this->get_logger(), "文件已安全关闭。");
        }
    }

private:
    void force_callback(const rm_ros_interfaces::msg::Sixforce::SharedPtr msg) {
        // 获取当前 ROS 2 系统时间戳（纳秒级）
        auto stamp = this->get_clock()->now().nanoseconds();

        if (file_out_.is_open()) {
        // 写入：时间戳, Fx, Fy, Fz, Tx, Ty, Tz
        file_out_ << stamp << ","
                            << msg->force_fx << "," 
                            << msg->force_fy << "," 
                            << msg->force_fz << ","
                            << msg->force_mx << "," 
                            << msg->force_my << "," 
                            << msg->force_mz
                            << std::endl;
        }
    }

    std::string filename_;
    std::ofstream file_out_;
    rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForceCollector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}