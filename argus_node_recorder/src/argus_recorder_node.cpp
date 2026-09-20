
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <argus_msgs/msg/gauge_state.hpp>
#include <argus_msgs/msg/obstacle_array.hpp>

namespace argus {

using ObstacleArray = argus_msgs::msg::ObstacleArray;
using GaugeState = argus_msgs::msg::GaugeState;

class EventLog {
public:
    EventLog(const std::string& dir, uint64_t max_file_bytes, uint32_t max_files)
        : dir_(dir), max_file_bytes_(max_file_bytes), max_files_(max_files) {
        std::error_code ec;
        std::filesystem::create_directories(dir_, ec);
    }

    void write(const std::string& line) {
        if (!stream_.is_open() || written_ >= max_file_bytes_) {
            rotate();
        }
        stream_ << line << '\n';
        written_ += line.size() + 1;
    }

    void flush() {
        if (stream_.is_open()) {
            stream_.flush();
        }
    }

    uint32_t files_written() const { return files_written_; }

private:
    void rotate() {
        if (stream_.is_open()) {
            stream_.close();
        }
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        char name[256];
        std::snprintf(name, sizeof(name), "%s/events_%lld.jsonl", dir_.c_str(),
                      static_cast<long long>(ms));
        stream_.open(name, std::ios::out | std::ios::trunc);
        written_ = 0;
        ++files_written_;
        if (files_written_ > 1) {
            prune_old();
        }
    }

    void prune_old() {
        std::vector<std::filesystem::directory_entry> files;
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(dir_, ec)) {
            const auto name = entry.path().filename().string();
            if (name.rfind("events_", 0) == 0 && entry.path().extension() == ".jsonl") {
                files.push_back(entry);
            }
        }
        std::sort(files.begin(), files.end(), [](const auto& a, const auto& b) {
            return a.path().filename() > b.path().filename();
        });
        for (size_t i = max_files_; i < files.size(); ++i) {
            std::filesystem::remove(files[i].path(), ec);
        }
    }

    std::string dir_;
    uint64_t max_file_bytes_;
    uint32_t max_files_;
    std::ofstream stream_;
    uint64_t written_ = 0;
    uint32_t files_written_ = 0;
};

class RecorderNode : public rclcpp::Node {
public:
    RecorderNode() : Node("argus_recorder") {
        const std::string dir =
            declare_parameter<std::string>("recorder.output_dir", "/tmp/argus_events");
        const uint64_t max_mb = static_cast<uint64_t>(
            std::max<int>(declare_parameter<int>("recorder.max_file_mb", 64), 1));
        const uint32_t max_files = static_cast<uint32_t>(
            std::max<int>(declare_parameter<int>("recorder.max_files", 24), 1));
        flush_period_s_ = declare_parameter<double>("recorder.flush_period_s", 1.0);
        if (flush_period_s_ <= 0.0) {
            throw std::invalid_argument("recorder.flush_period_s must be > 0");
        }
        log_ = std::make_unique<EventLog>(dir, max_mb * 1024 * 1024, max_files);

        auto qos = rclcpp::QoS(10);
        sub_obstacles_ = create_subscription<ObstacleArray>(
            "/argus/obstacles", qos,
            [this](ObstacleArray::ConstSharedPtr msg) { on_obstacles(msg); });
        sub_gauge_ = create_subscription<GaugeState>(
            "/argus/gauge_state", qos, [this](GaugeState::ConstSharedPtr msg) {
                char line[320];
                std::snprintf(line, sizeof(line),
                              "{\"type\":\"gauge\",\"stamp\":%u.%09u,\"speed\":%.2f,"
                              "\"braking_m\":%.1f,\"clear_m\":%.1f,\"limit_mps\":%.2f}",
                              msg->header.stamp.sec, msg->header.stamp.nanosec,
                              static_cast<double>(msg->train_speed_mps),
                              static_cast<double>(msg->braking_distance_m),
                              static_cast<double>(msg->clear_range_m),
                              static_cast<double>(msg->speed_limit_mps));
                log_->write(line);
            });
        sub_health_ = create_subscription<std_msgs::msg::String>(
            "/argus/system_health", qos, [this](std_msgs::msg::String::ConstSharedPtr msg) {
                if (msg->data == last_health_) {
                    return;
                }
                last_health_ = msg->data;
                char line[256];
                std::snprintf(line, sizeof(line), "{\"type\":\"health\",\"value\":\"%s\"}",
                              msg->data.c_str());
                log_->write(line);
            });
        flush_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
                                             std::chrono::duration<double>(flush_period_s_)),
                                         [this] { log_->flush(); });

        RCLCPP_INFO(get_logger(), "argus_recorder: журнал в %s (файл <= %llu МБ, файлов <= %u)",
                    dir.c_str(), static_cast<unsigned long long>(max_mb), max_files);
    }

private:
    static const char* status_name(uint8_t status) {
        switch (status) {
            case ObstacleArray::STATUS_BLOCKED:
                return "BLOCKED";
            case ObstacleArray::STATUS_WARNING:
                return "WARNING";
            case ObstacleArray::STATUS_DEGRADED:
                return "DEGRADED";
            default:
                return "CLEAR";
        }
    }

    void on_obstacles(const ObstacleArray::ConstSharedPtr& msg) {
        const bool interesting =
            msg->status != ObstacleArray::STATUS_CLEAR || msg->status != last_status_;
        if (!interesting) {
            return;
        }
        last_status_ = msg->status;
        std::string line;
        char head[320];
        std::snprintf(head, sizeof(head),
                      "{\"type\":\"decision\",\"stamp\":%u.%09u,\"status\":\"%s\","
                      "\"nearest_m\":%.2f,\"obstacles\":%u,\"candidates\":%zu,"
                      "\"processing_ms\":%.1f,\"fps\":%.1f",
                      msg->header.stamp.sec, msg->header.stamp.nanosec, status_name(msg->status),
                      static_cast<double>(msg->nearest_range_m), msg->obstacles_detected,
                      msg->candidates.size(), static_cast<double>(msg->processing_ms),
                      static_cast<double>(msg->fps));
        line = head;
        if (!msg->obstacles.empty()) {
            line += ",\"tracks\":[";
            for (size_t i = 0; i < msg->obstacles.size(); ++i) {
                const auto& o = msg->obstacles[i];
                char ob[256];
                std::snprintf(ob, sizeof(ob),
                              "%s{\"id\":%u,\"range_m\":%.2f,\"ttc_s\":%.2f,\"severity\":%u}",
                              i == 0 ? "" : ",", o.track_id, static_cast<double>(o.range_m),
                              static_cast<double>(o.ttc_s), o.severity);
                line += ob;
            }
            line += "]";
        }
        line += "}";
        log_->write(line);
    }

    std::unique_ptr<EventLog> log_;
    double flush_period_s_ = 1.0;
    uint8_t last_status_ = ObstacleArray::STATUS_CLEAR;
    std::string last_health_;
    rclcpp::Subscription<ObstacleArray>::SharedPtr sub_obstacles_;
    rclcpp::Subscription<GaugeState>::SharedPtr sub_gauge_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_health_;
    rclcpp::TimerBase::SharedPtr flush_timer_;
};

} // namespace argus

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<argus::RecorderNode>());
    rclcpp::shutdown();
    return 0;
}
