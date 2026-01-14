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

#ifndef GNSSSERIALLOADER_H
#define GNSSSERIALLOADER_H

#include <string>
#include <vector>

#include "common/types.h"
#include "serialport.h"

class GnssSerialLoader {
public:
    GnssSerialLoader(const std::string& port, int baudrate);

    ~GnssSerialLoader() = default;

    const GNSS& next();

    bool isOpen() const;

private:
    SerialPort serial_;
    GNSS gnss_;
    std::string buffer_;
    bool is_open_{false};
    
    // 追加数据到缓冲区
    void append(const char* data, int size);
    
    // 解析NMEA数据
    bool parseNMEA(const std::string& nmea);
    
    // 分割字符串
    std::vector<std::string> split(const std::string& str, char delimiter);
};

#endif // GNSSSERIALLOADER_H
