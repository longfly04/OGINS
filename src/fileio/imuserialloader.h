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
#include "serialport.h"

class ImuSerialLoader {
public:
    ImuSerialLoader(const std::string& port, int baudrate, int rate = 200);

    ~ImuSerialLoader() = default;

    const IMU& next();

    bool isOpen() const;

private:
    SerialPort serial_;
    IMU imu_, imu_pre_;
    std::string buffer_;
    bool is_open_{false};
    double dt_;
    
    // 追加数据到缓冲区
    void append(const char* data, int size);
    
    // 解析IMU数据
    bool parseIMUData(const std::string& data);
    
    // 分割字符串
    std::vector<std::string> split(const std::string& str, char delimiter);
};

#endif // IMUSERIALLOADER_H
