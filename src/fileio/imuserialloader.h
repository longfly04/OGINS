/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Your Name
 *    Contact : your.email@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef IMUSERIALLOADER_H
#define IMUSERIALLOADER_H

#include <string>
#include <vector>

#include "common/types.h"
#include "common/logger.h"
#include "serialport.h"

class ImuSerialLoader {
public:
    ImuSerialLoader(const std::string& port, int baudrate, int rate = 200);

    ~ImuSerialLoader() = default;

    const IMU& next();

    bool isOpen() const;

private:
    // IMU数据缓存结构体
    struct IMUDataCache {
        Vector3d acceleration;    // 加速度 [m/s^2]
        Vector3d angular_velocity; // 角速度 [rad/s]
        Vector3d angle;           // 角度 [rad]
        Vector3d magnetometer;     // 磁场 [uT]
        bool has_acceleration{false};    // 加速度数据标志
        bool has_angular_velocity{false}; // 角速度数据标志
        bool has_angle{false};           // 角度数据标志
        bool has_magnetometer{false};     // 磁场数据标志
        double timestamp{0.0};           // 时间戳 [s]
        
        // 数据过滤相关
        Vector3d last_acceleration;      // 上一次加速度数据
        Vector3d last_angular_velocity;  // 上一次角速度数据
        int valid_data_count{0};         // 有效数据计数
    };

    SerialPort serial_;
    IMU imu_, imu_pre_;
    IMUDataCache imu_cache_;
    std::string buffer_;
    bool is_open_{false};
    double dt_;
    size_t parse_state_{0}; // 解析状态：0-寻找帧头，1-寻找数据类型，2-接收数据，3-接收校验和
    unsigned char current_data_type_{0}; // 当前数据类型
    std::vector<unsigned char> frame_buffer_; // 帧缓冲区
    double last_data_time_{0.0}; // 上次数据时间
    
    // 数据过滤参数
    double acc_threshold_{100.0}; // 加速度阈值 [m/s^2]
    double gyro_threshold_{100.0}; // 角速度阈值 [rad/s]
    
    // 异常处理参数
    int max_error_count_{100}; // 最大连续错误数
    double timeout_{1.0}; // 超时时间 [s]
    
    // 状态统计
    int frame_count_{0}; // 总帧数
    int error_count_{0}; // 错误帧数
    int valid_frame_count_{0}; // 有效帧数
    int drop_frame_count_{0}; // 丢帧数
    
    // 追加数据到缓冲区
    void append(const char* data, int size);
    
    // 解析IMU数据（二进制协议）
    bool parseIMUData();
    
    // 校验和验证
    bool verifyChecksum(const std::vector<unsigned char>& frame);
    
    // 解析16位数据
    short parse16Bit(const unsigned char* data);
    
    // 数据过滤
    bool filterData(Vector3d& acceleration, Vector3d& angular_velocity);
    
    // 重置状态
    void resetState();
    
    // 清理缓冲区
    void clearBuffer();
};

#endif // IMUSERIALLOADER_H
