#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <robotiq_driver/driver.hpp>
#include <robotiq_driver/default_driver.hpp>
#include <robotiq_driver/default_serial.hpp>
#include <robotiq_driver/fake/fake_driver.hpp>

#include <zerolancom/zerolancom.hpp>
#include "utils/atomic_double_buffer.hpp"
#include "utils/config_file_reader.hpp"
struct GripperStateMSg {
    double position; // 0.0 .. 1.0
    bool is_moving;
    uint8_t raw_position;//0 .. 255
    int64_t time;

    MSGPACK_DEFINE_MAP(position, is_moving, raw_position, time)
};

struct GraspCommand {
    double position; // 0.0 .. 1.0
    double speed;    // 0.0 .. 1.0
    double force;    // 0.0 .. 1.0

    GraspCommand() : position(0.0), speed(0.0), force(0.0) {}
    MSGPACK_DEFINE_MAP(position, speed, force)
};



struct RobotiqGripperConfig {
    std::string node_name;
    std::string proxy_ip;
    std::string group;
    int group_port = 0;
    std::string group_name;
    std::string serial_port;
    uint8_t slave_address = 0x09;
    uint32_t baudrate = 115200;
    std::string command_topic;
    std::string state_topic;
    int gripper_pub_rate_hz = 100;

    RobotiqGripperConfig(const std::string& config_path) { fromFile(config_path); }

    void fromFile(const std::string& config_path) {
        ConfigFileReader reader(config_path);
        node_name = reader.getValue<std::string>("node_name", std::string{});
        proxy_ip = reader.getValue<std::string>("proxy_ip", std::string{});
        group = reader.getValue<std::string>("group", std::string{});
        group_port = reader.getValue<int>("group_port", 0);
        group_name = reader.getValue<std::string>("group_name", std::string{});
        serial_port = reader.getValue<std::string>("serial_port");
        slave_address = reader.getValue<uint8_t>("slave_address");
        baudrate = reader.getValue<uint32_t>("baudrate");
        command_topic = reader.getValue<std::string>("command_topic");
        state_topic = reader.getValue<std::string>("state_topic");
        gripper_pub_rate_hz = reader.getValue<int>("GRIPPER_PUB_RATE_HZ", 100);
    }
};

class RobotiqGripperProxy {
public:
    explicit RobotiqGripperProxy(const std::string& config_path): 
    is_running(false), 
    command_(AtomicDoubleBuffer<RobotiqGraspCommand>(GraspCommand{})), 
    config_(config_path)
    {
#ifndef NO_ROBOT_TESTING
        // Create real serial and driver
        auto serial = std::make_unique<robotiq_driver::DefaultSerial>();
        serial->set_port(config_.serial_port);
        serial->set_baudrate(config_.baudrate);
        driver_ = std::make_unique<robotiq_driver::DefaultDriver>(std::move(serial));
        driver_->set_slave_address(config_.slave_address);
        try {
            if (!driver_->connect()) {
                zlc::warn("Robotiq driver failed to connect to {}", config_.serial_port);
            }
            else {
                driver_->activate();
            }
        }
        catch (const std::exception& e) {
            zlc::error("Robotiq driver exception during connect/activate: {}", e.what());
        }
#else
        driver_ = std::make_unique<robotiq_driver::FakeDriver>();
        driver_->set_slave_address(config_.slave_address);
        driver_->connect();
        driver_->activate();
#endif
        command_.write(GraspCommand{0.0f, 0.1f, 0.1f}); //reset command
        is_running = true;
        zlc::info("Gripper proxy running flag set to {}", is_running.load());
        zlc::registerSubscriberHandler(config_.command_topic, &RobotiqGripperProxy::updateCommand, this);
        state_pub_thread_ = std::thread(&RobotiqGripperProxy::statePubThread, this);
        control_thread_ = std::thread(&RobotiqGripperProxy::controlLoopThread, this);
    }

    ~RobotiqGripperProxy() { 
        stop(); 
    };

    void stop() {
        zlc::info("Stopping RobotiqGripperProxy...");
        is_running = false;
        if (state_pub_thread_.joinable()) state_pub_thread_.join();
        if (control_thread_.joinable()) control_thread_.join();
        command_.write(GraspCommand{0.0f, 0.1f, 0.1f}); //reset command
        if (driver_) {
            try { driver_->deactivate(); driver_->disconnect(); }
            catch (...) {}
        }
        zlc::info("RobotiqGripperProxy stopped.");
    }

private:
    std::unique_ptr<robotiq_driver::Driver> driver_;
    std::thread state_pub_thread_;
    std::thread control_thread_;

    void controlLoopThread() {
        while (is_running) {
            GraspCommand cmd = command_.read();
            try {
                uint8_t pos = static_cast<uint8_t>(std::round(std::clamp(cmd.position, 0.0, 1.0) * 255.0));
                uint8_t speed = static_cast<uint8_t>(std::round(std::clamp(cmd.speed, 0.0, 1.0) * 255.0));
                uint8_t force = static_cast<uint8_t>(std::round(std::clamp(cmd.force, 0.0, 1.0) * 255.0));
                driver_->set_speed(speed);
                driver_->set_force(force);
                driver_->set_gripper_position(pos);
            }
            catch (const std::exception& e) {
                zlc::error("RobotiqGripperProxy control exception: {}", e.what());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void statePubThread() {
        zlc::info("RobotiqGripperProxy state publishing thread started.");
        const std::string topic_name = fmt::format("{}/{}", config_.name, config_.state_topic);
        zlc::Publisher<RobotiqGripperState> state_publisher(topic_name);
        while (is_running) {
            try {
                uint8_t raw = driver_->get_gripper_position();
                bool moving = driver_->gripper_is_moving();
                const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count();
                GripperState gsm{static_cast<double>(raw) / 255.0, moving, raw, now_ms};//first is position 0.0 .. 1.0
                state_publisher.publish(gsm);
            }
            catch (const std::exception& e) {
                zlc::error("RobotiqGripperProxy state thread exception: {}", e.what());
            }
             std::this_thread::sleep_until(
                std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / gripper_pub_rate_hz)
            );
        }
    }

    void updateCommand(const RobotiqGraspCommand& cmd) { 
        command_.write(cmd); 
    };

    std::atomic<bool> is_running;
    AtomicDoubleBuffer<GraspCommand> command_;
    RobotiqGripperConfig config_;

    // static constexpr int GRIPPER_PUB_RATE_HZ = 100;
};
