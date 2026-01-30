#include <iostream>
#include <thread>
#include <chrono>
#include <fmt/format.h>

#include "robotiq_gripper_proxy.hpp"
#include <zerolancom/zerolancom.hpp>

int main(int argc, char** argv)
{
    // Config path (can be overridden by first arg)
    std::string config_path = "config/RobotiqGripperConfig.yaml";
    if (argc > 1) config_path = argv[1];

    // Load config for topic names
    RobotiqGripperConfig cfg(config_path);
    if (!cfg.node_name.empty() && !cfg.proxy_ip.empty()) {
        zlc::init(cfg.node_name, cfg.proxy_ip, cfg.group, cfg.group_port, cfg.group_name);
    } else {
        zlc::init("robotiq_example", "127.0.0.1");
    }
    const std::string command_topic = fmt::format("{}/{}", cfg.name, cfg.command_topic);
    const std::string state_topic = fmt::format("{}/{}", cfg.name, cfg.state_topic);

    // Create the proxy (it will start threads and register handlers)
    RobotiqGripperProxy proxy(config_path);

    // Subscribe to state updates
    zlc::registerSubscriberHandler(
        state_topic,
        static_cast<void (*)(const RobotiqGripperState&)>([](const RobotiqGripperState& s) {
            std::cout << "State: pos=" << s.position << " raw=" << int(s.raw_position)
                      << " moving=" << s.is_moving << " time=" << s.time << std::endl;
        }));

    // Publisher for commands
    zlc::Publisher<GraspCommand> cmd_pub(command_topic);

    // Send a few commands
    GraspCommand cmd;
    std::cout << "Closing gripper (position=1.0)" << std::endl;
    cmd.position = 1.0;
    cmd.speed = 0.5;
    cmd.force = 0.8;
    cmd_pub.publish(cmd);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "Opening gripper (position=0.0)" << std::endl;
    cmd.position = 0.0;
    cmd_pub.publish(cmd);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    proxy.stop();
    zlc::shutdown();
    return 0;
}
