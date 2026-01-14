#include "fileio/imuserialloader.h"

#include <iostream>
#include <sstream>
#include <vector>

#include <Eigen/Geometry>

using Eigen::Vector3d;

ImuSerialLoader::ImuSerialLoader(const std::string& port, int baudrate, int rate) {
    if (!serial_.open(port, baudrate)) {
        is_open_ = false;
    } else {
        is_open_ = true;
        dt_ = 1.0 / (double)rate;
        imu_.time = 0;
        imu_pre_.time = 0;
    }
}

const IMU& ImuSerialLoader::next() {
    imu_.time = 0;
    imu_.dt = 0;
    imu_.dtheta = Vector3d::Zero();
    imu_.dvel = Vector3d::Zero();
    imu_.odovel = 0;

    char buffer[1024] = {0};
    int bytes_read = serial_.read(buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        append(buffer, bytes_read);

        size_t end_idx = buffer_.find('\n');
        while (end_idx != std::string::npos) {
            std::string line = buffer_.substr(0, end_idx);
            buffer_ = buffer_.substr(end_idx + 1);
            if (parseIMUData(line)) {
                break;
            }
            end_idx = buffer_.find('\n');
        }
    }

    return imu_;
}

bool ImuSerialLoader::isOpen() const {
    return is_open_;
}

void ImuSerialLoader::append(const char* data, int size) {
    buffer_.append(data, size);
    // 限制缓冲区大小，防止内存溢出
    if (buffer_.size() > 4096) {
        buffer_ = buffer_.substr(buffer_.size() - 4096);
    }
}

bool ImuSerialLoader::parseIMUData(const std::string& line) {
    std::istringstream iss(line);
    std::string token;
    std::vector<double> data;

    while (std::getline(iss, token, ',')) {
        try {
            data.push_back(std::stod(token));
        } catch (...) {
            return false;
        }
    }

    if (data.size() >= 7) {
        imu_pre_ = imu_;
        
        // 解析时间
        imu_.time = data[0];
        
        // 解析角增量
        imu_.dtheta[0] = data[1];
        imu_.dtheta[1] = data[2];
        imu_.dtheta[2] = data[3];
        
        // 解析速度增量
        imu_.dvel[0] = data[4];
        imu_.dvel[1] = data[5];
        imu_.dvel[2] = data[6];
        
        // 计算时间间隔
        double dt = imu_.time - imu_pre_.time;
        if (dt > 0 && dt < 0.1) {
            imu_.dt = dt;
        } else {
            imu_.dt = dt_;
        }
        
        return true;
    }

    return false;
}

std::vector<std::string> ImuSerialLoader::split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    for (char c : str) {
        if (c == delimiter) {
            tokens.push_back(token);
            token.clear();
        } else {
            token += c;
        }
    }
    tokens.push_back(token);
    return tokens;
}
