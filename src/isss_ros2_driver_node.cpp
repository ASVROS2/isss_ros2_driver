
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "smarc_msgs/msg/sidescan.hpp"

typedef unsigned short U16;
typedef unsigned char U8;

using namespace std;

class ISSSonarNode : public rclcpp::Node
{
public:
    ISSSonarNode() : Node("isss_ros2_driver_node")
    {
        this->declare_parameter<std::string>("save_path", "/root/ros2_ws/src/isss_ros2_driver/isss_data/");
        this->declare_parameter<double>("range", 20.0);

        this->get_parameter("save_path", save_path_);
        this->get_parameter("range", m_range_);

        num_samples_ = 1000;

        sidescan_pub_ = this->create_publisher<smarc_msgs::msg::Sidescan>("isss_stick/isss/raw", 10);

        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        serverAddr_.sin_family = AF_INET;
        serverAddr_.sin_port = htons(0xE005);
        inet_pton(AF_INET, "192.168.2.62", &serverAddr_.sin_addr);

        if (connect(sock_, (sockaddr*)&serverAddr_, sizeof(serverAddr_)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connect() error: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        // Set receive timeout
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

        RCLCPP_INFO(this->get_logger(), "Connected to sonar, setting up...");

        SetupSonar();

        // Prepare output file
        std::string filename = save_path_ + std::to_string(this->now().seconds()) + ".bin";
        os_.open(filename, std::ios::binary);

        RCLCPP_INFO(this->get_logger(), "Saving sonar data to: %s", filename.c_str());

        // Start reading loop in a separate thread:
        read_thread_ = std::thread(&ISSSonarNode::ReadLoop, this);
    }

    ~ISSSonarNode() {
        if (sock_ >= 0) close(sock_);
        if (read_thread_.joinable()) read_thread_.join();
        if (os_.is_open()) os_.close();
    }

private:
    void SetupSonar() {
        int decimation = int(float(m_range_) / num_samples_ / 750 * (12.5e6f / 3));
        SetReg_V4(0x10, decimation);
        SetReg_V4(0x11, num_samples_);
        SetReg_V4(0x12, 2); // Num Phases
    }

    void SetReg_V4(int reg, int value) {
        U8 txBuff[4];
        txBuff[0] = 0xFE;
        txBuff[1] = U8(reg & 0xFF);
        txBuff[2] = U8((value >> 8) & 0xFF);
        txBuff[3] = U8(value & 0xFF);
        sendto(sock_, (const char*)txBuff, 4, 0, (sockaddr*)&serverAddr_, sizeof(serverAddr_));
    }

    void ReadLoop() {
        const int HEADER_SIZE = 12;
        const int DATA_SIZE = 6000;
        const int PACKET_SIZE = HEADER_SIZE + DATA_SIZE;

        char headerBuffer[HEADER_SIZE];
        char dataBuffer[DATA_SIZE];
        char packetBuffer[PACKET_SIZE];

        smarc_msgs::msg::Sidescan msg;
        msg.max_duration = float(m_range_ / num_samples_);
        msg.port_channel.resize(num_samples_);
        msg.starboard_channel.resize(num_samples_);
        msg.port_channel_angle_high.resize(num_samples_);
        msg.port_channel_angle_low.resize(num_samples_);
        msg.starboard_channel_angle_high.resize(num_samples_);
        msg.starboard_channel_angle_low.resize(num_samples_);
        msg.extra_channel.resize(12);

        bool headerFound = false;

        while (rclcpp::ok()) {
            // Read header
            while (!headerFound && rclcpp::ok()) {
                char byte;
                int bytesReceived = recv(sock_, &byte, 1, 0);
                if (bytesReceived <= 0) continue;

                memmove(headerBuffer, headerBuffer + 1, HEADER_SIZE - 1);
                headerBuffer[HEADER_SIZE - 1] = byte;

                if ((U8)headerBuffer[0] == 0xFE &&
                    (U8)headerBuffer[1] == 0x05 &&
                    (U8)headerBuffer[4] == 0xE1 &&
                    (U8)headerBuffer[9] == 0x01) {
                    headerFound = true;
                    memcpy(packetBuffer, headerBuffer, HEADER_SIZE);
                }
            }

            if (!rclcpp::ok()) break;

            // Read data
            int totalBytesRead = 0;
            while (totalBytesRead < DATA_SIZE && rclcpp::ok()) {
                int bytesReceived = recv(sock_, dataBuffer + totalBytesRead, DATA_SIZE - totalBytesRead, 0);
                if (bytesReceived <= 0) {
                    headerFound = false;
                    break;
                }
                totalBytesRead += bytesReceived;
            }

            if (!rclcpp::ok()) break;
            if (totalBytesRead != DATA_SIZE) continue;

            memcpy(packetBuffer + HEADER_SIZE, dataBuffer, DATA_SIZE);
            os_.write(packetBuffer, PACKET_SIZE);

            // Fill ROS2 message
            msg.header.stamp = this->now();

            for (size_t i = 0; i < 12; ++i) msg.extra_channel[i] = (U8)packetBuffer[i];
            for (size_t i = 0; i < num_samples_; ++i) {
                size_t offset = HEADER_SIZE + i * 6;
                msg.port_channel_angle_high[i] = (U8)packetBuffer[offset];
                msg.port_channel_angle_low[i] = (U8)packetBuffer[offset + 1];
                msg.port_channel[i] = (U8)packetBuffer[offset + 2];
                msg.starboard_channel_angle_high[i] = (U8)packetBuffer[offset + 3];
                msg.starboard_channel_angle_low[i] = (U8)packetBuffer[offset + 4];
                msg.starboard_channel[i] = (U8)packetBuffer[offset + 5];
            }

            sidescan_pub_->publish(msg);
            headerFound = false;
        }
    }

    rclcpp::Publisher<smarc_msgs::msg::Sidescan>::SharedPtr sidescan_pub_;
    std::string save_path_;
    double m_range_;
    //int num_samples_;
    size_t num_samples_;
    int sock_;
    struct sockaddr_in serverAddr_;
    std::ofstream os_;
    std::thread read_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ISSSonarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
