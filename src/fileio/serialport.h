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

#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    bool open(const std::string& port, int baudrate);
    void close();
    bool isOpen() const;
    int read(char* buffer, int size);
    int write(const char* buffer, int size);

private:
    void* hCom_{nullptr};
    bool is_open_{false};
};

#endif // SERIALPORT_H
