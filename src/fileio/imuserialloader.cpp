#include "fileio/imuserialloader.h"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

using Eigen::Vector3d;

ImuSerialLoader::ImuSerialLoader(const std::string& port, int baudrate, int rate) {
    if (!serial_.open(port, baudrate)) {
        is_open_ = false;
        LOG_ERROR("Failed to open serial port: %s, baudrate: %d", port.c_str(), baudrate);
    } else {
        is_open_ = true;
        dt_ = 1.0 / (double)rate;
        imu_.time = 0;
        imu_pre_.time = 0;
        
        // 初始化状态
        parse_state_ = 0;
        current_data_type_ = 0;
        frame_buffer_.reserve(11); // 帧大小固定为11字节
        
        // 初始化数据缓存
        imu_cache_.last_acceleration.setZero();
        imu_cache_.last_angular_velocity.setZero();
        
        LOG_INFO("IMU serial loader initialized successfully: port=%s, baudrate=%d, rate=%d Hz", 
                port.c_str(), baudrate, rate);
    }
}

const IMU& ImuSerialLoader::next() {
    imu_.time = 0;
    imu_.dt = 0;
    imu_.dtheta = Vector3d::Zero();
    imu_.dvel = Vector3d::Zero();
    imu_.odovel = 0;

    // 检查超时
    double current_time = imu_cache_.timestamp;
    if (current_time - last_data_time_ > timeout_ && last_data_time_ > 0) {
        LOG_WARN("IMU data timeout detected: %f s", current_time - last_data_time_);
        resetState();
    }

    // 读取串口数据
    char buffer[1024] = {0};
    int bytes_read = serial_.read(buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        append(buffer, bytes_read);
        LOG_DEBUG("Read %d bytes from serial port", bytes_read);
        
        // 解析二进制数据
        if (parseIMUData()) {
            last_data_time_ = current_time;
        } else {
            // 检查连续错误数
            error_count_++;
            if (error_count_ > max_error_count_) {
                LOG_ERROR("Too many consecutive errors (%d), resetting state", error_count_);
                resetState();
            }
        }
    }

    return imu_;
}

bool ImuSerialLoader::isOpen() const {
    return is_open_;
}

void ImuSerialLoader::append(const char* data, int size) {
    buffer_.append(data, size);
    
    // 优化缓冲区管理：当缓冲区超过最大限制时，只保留后面的数据
    if (buffer_.size() > 4096) {
        // 保留最新的4096字节数据，确保不丢失可能的完整帧
        buffer_ = buffer_.substr(buffer_.size() - 4096);
        LOG_DEBUG("Buffer size exceeded 4096 bytes, trimming to 4096 bytes");
    }
    
    LOG_DEBUG("Buffer size after append: %zu bytes", buffer_.size());
}

bool ImuSerialLoader::parseIMUData() {
    size_t pos = 0;
    bool parsed = false;
    
    // 优化：一次性处理整个缓冲区，提高解析效率
    while (pos + 11 <= buffer_.size()) {
        frame_count_++;
        
        // 快速帧头检测
        if (static_cast<unsigned char>(buffer_[pos]) != 0x55) {
            pos++;
            continue;
        }
        
        // 检查数据类型
        unsigned char data_type = static_cast<unsigned char>(buffer_[pos + 1]);
        if (data_type < 0x51 || data_type > 0x54) {
            pos++;
            continue;
        }
        
        // 检查帧长度
        if (pos + 11 > buffer_.size()) {
            break;
        }
        
        // 提取完整帧
        frame_buffer_.assign(buffer_.begin() + pos, buffer_.begin() + pos + 11);
        
        // 验证校验和
        if (verifyChecksum(frame_buffer_)) {
            valid_frame_count_++;
            error_count_ = 0; // 重置错误计数
            
            const unsigned char* data = frame_buffer_.data();
            
            switch (data_type) {
                case 0x51: { // 加速度数据
                    short ax = parse16Bit(data + 2);
                    short ay = parse16Bit(data + 4);
                    short az = parse16Bit(data + 6);
                    
                    // 转换为m/s^2
                    imu_cache_.acceleration[0] = ax / 32768.0 * 16 * 9.8;
                    imu_cache_.acceleration[1] = ay / 32768.0 * 16 * 9.8;
                    imu_cache_.acceleration[2] = az / 32768.0 * 16 * 9.8;
                    imu_cache_.has_acceleration = true;
                    LOG_DEBUG("Parsed acceleration data: %.3f, %.3f, %.3f m/s^2", 
                            imu_cache_.acceleration[0], imu_cache_.acceleration[1], imu_cache_.acceleration[2]);
                    break;
                }
                
                case 0x52: { // 角速度数据
                    short wx = parse16Bit(data + 2);
                    short wy = parse16Bit(data + 4);
                    short wz = parse16Bit(data + 6);
                    
                    // 转换为rad/s
                    imu_cache_.angular_velocity[0] = wx / 32768.0 * 2000 * M_PI / 180;
                    imu_cache_.angular_velocity[1] = wy / 32768.0 * 2000 * M_PI / 180;
                    imu_cache_.angular_velocity[2] = wz / 32768.0 * 2000 * M_PI / 180;
                    imu_cache_.has_angular_velocity = true;
                    LOG_DEBUG("Parsed angular velocity data: %.3f, %.3f, %.3f rad/s", 
                            imu_cache_.angular_velocity[0], imu_cache_.angular_velocity[1], imu_cache_.angular_velocity[2]);
                    break;
                }
                
                case 0x53: { // 角度数据
                    short rx = parse16Bit(data + 2);
                    short ry = parse16Bit(data + 4);
                    short rz = parse16Bit(data + 6);
                    
                    // 转换为rad
                    imu_cache_.angle[0] = rx / 32768.0 * 180 * M_PI / 180;
                    imu_cache_.angle[1] = ry / 32768.0 * 180 * M_PI / 180;
                    imu_cache_.angle[2] = rz / 32768.0 * 180 * M_PI / 180;
                    imu_cache_.has_angle = true;
                    LOG_DEBUG("Parsed angle data: %.3f, %.3f, %.3f rad", 
                            imu_cache_.angle[0], imu_cache_.angle[1], imu_cache_.angle[2]);
                    break;
                }
                
                case 0x54: { // 磁场数据
                    short mx = parse16Bit(data + 2);
                    short my = parse16Bit(data + 4);
                    short mz = parse16Bit(data + 6);
                    
                    imu_cache_.magnetometer[0] = mx;
                    imu_cache_.magnetometer[1] = my;
                    imu_cache_.magnetometer[2] = mz;
                    imu_cache_.has_magnetometer = true;
                    LOG_DEBUG("Parsed magnetometer data: %d, %d, %d uT", 
                            static_cast<int>(imu_cache_.magnetometer[0]), 
                            static_cast<int>(imu_cache_.magnetometer[1]), 
                            static_cast<int>(imu_cache_.magnetometer[2]));
                    break;
                }
            }
            
            // 当收集到足够的数据后，生成IMU结构体
            if (imu_cache_.has_acceleration && imu_cache_.has_angular_velocity) {
                // 数据过滤
                Vector3d filtered_acc = imu_cache_.acceleration;
                Vector3d filtered_gyro = imu_cache_.angular_velocity;
                
                if (filterData(filtered_acc, filtered_gyro)) {
                    imu_pre_ = imu_;
                    
                    // 更新时间
                    imu_cache_.timestamp += dt_;
                    imu_.time = imu_cache_.timestamp;
                    
                    // 计算角速度增量和速度增量
                    imu_.dtheta = filtered_gyro * dt_;
                    imu_.dvel = filtered_acc * dt_;
                    
                    // 计算时间间隔
                    double dt = imu_.time - imu_pre_.time;
                    if (dt > 0 && dt < 0.1) {
                        imu_.dt = dt;
                    } else {
                        imu_.dt = dt_;
                    }
                    
                    parsed = true;
                    LOG_DEBUG("Generated IMU data at time: %.3f, dt: %.6f", imu_.time, imu_.dt);
                    
                    // 重置数据标志
                    imu_cache_.has_acceleration = false;
                    imu_cache_.has_angular_velocity = false;
                    
                    // 统计信息
                    if (imu_cache_.valid_data_count % 100 == 0) {
                        LOG_INFO("IMU Parser Stats: Total frames: %d, Valid: %d, Errors: %d, Drop: %d", 
                                frame_count_, valid_frame_count_, error_count_, drop_frame_count_);
                    }
                    imu_cache_.valid_data_count++;
                    
                    // 更新最后数据时间
                    last_data_time_ = imu_cache_.timestamp;
                    
                    break;
                } else {
                    drop_frame_count_++;
                    LOG_WARN("Filtered out invalid IMU data, drop count: %d", drop_frame_count_);
                    // 重置数据标志，继续解析
                    imu_cache_.has_acceleration = false;
                    imu_cache_.has_angular_velocity = false;
                }
            }
            
            // 跳过已处理的帧
            pos += 11;
        } else {
            error_count_++;
            LOG_WARN("Checksum verification failed, error count: %d", error_count_);
            pos++;
            
            // 检查连续错误数
            if (error_count_ > max_error_count_) {
                LOG_ERROR("Too many consecutive errors (%d), resetting parser", error_count_);
                resetState();
                parsed = false;
                break;
            }
        }
    }
    
    // 优化：移动未处理的数据到缓冲区开头，避免重复处理
    if (pos > 0 && pos < buffer_.size()) {
        buffer_ = buffer_.substr(pos);
        LOG_DEBUG("Buffer trimmed, new size: %zu bytes", buffer_.size());
    } else if (pos >= buffer_.size()) {
        buffer_.clear();
        LOG_DEBUG("Buffer cleared");
    }
    
    return parsed;
}

bool ImuSerialLoader::verifyChecksum(const std::vector<unsigned char>& frame) {
    if (frame.size() != 11) {
        return false;
    }
    
    unsigned char sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += frame[i];
    }
    
    return sum == frame[10];
}

short ImuSerialLoader::parse16Bit(const unsigned char* data) {
    // 小端模式：低字节在前，高字节在后
    return static_cast<short>((data[1] << 8) | data[0]);
}

bool ImuSerialLoader::filterData(Vector3d& acceleration, Vector3d& angular_velocity) {
    // 1. 阈值过滤：检查数据是否在合理范围内
    if (acceleration.norm() > acc_threshold_ || 
        angular_velocity.norm() > gyro_threshold_) {
        LOG_WARN("IMU data out of range: acc_norm=%.3f, gyro_norm=%.3f", 
                acceleration.norm(), angular_velocity.norm());
        return false;
    }
    
    // 2. 跳变检测：检查数据变化是否过大
    double acc_diff = (acceleration - imu_cache_.last_acceleration).norm();
    double gyro_diff = (angular_velocity - imu_cache_.last_angular_velocity).norm();
    
    // 允许的最大变化量（可根据实际情况调整）
    const double max_acc_jump = 10.0; // m/s^2
    const double max_gyro_jump = 5.0; // rad/s
    
    if (imu_cache_.valid_data_count > 0) {
        if (acc_diff > max_acc_jump || gyro_diff > max_gyro_jump) {
            LOG_WARN("IMU data jump detected: acc_diff=%.3f, gyro_diff=%.3f", 
                    acc_diff, gyro_diff);
            return false;
        }
    }
    
    // 3. 静态检测：检查是否为静态状态（可选）
    // 如果加速度接近重力，角速度接近0，则认为是静态
    const double static_acc_threshold = 1.0; // m/s^2
    const double static_gyro_threshold = 0.1; // rad/s
    
    double acc_from_gravity = acceleration.norm() - 9.8;
    if (fabs(acc_from_gravity) < static_acc_threshold && 
        angular_velocity.norm() < static_gyro_threshold) {
        LOG_DEBUG("Static state detected");
        // 可以在这里添加静态状态下的特殊处理
    }
    
    // 更新历史数据
    imu_cache_.last_acceleration = acceleration;
    imu_cache_.last_angular_velocity = angular_velocity;
    
    LOG_DEBUG("IMU data passed all filters");
    return true;
}

void ImuSerialLoader::resetState() {
    LOG_INFO("Resetting IMU parser state");
    
    // 重置解析状态
    parse_state_ = 0;
    current_data_type_ = 0;
    frame_buffer_.clear();
    
    // 重置数据缓存
    imu_cache_.has_acceleration = false;
    imu_cache_.has_angular_velocity = false;
    imu_cache_.has_angle = false;
    imu_cache_.has_magnetometer = false;
    imu_cache_.last_acceleration.setZero();
    imu_cache_.last_angular_velocity.setZero();
    imu_cache_.valid_data_count = 0;
    
    // 重置统计信息
    error_count_ = 0;
    
    // 清理缓冲区
    clearBuffer();
}

void ImuSerialLoader::clearBuffer() {
    buffer_.clear();
    LOG_DEBUG("Buffer cleared, buffer size: %zu bytes", buffer_.size());
}
