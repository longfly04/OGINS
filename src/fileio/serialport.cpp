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

#include "serialport.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#endif

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baudrate) {
#ifdef _WIN32
    // Windows 串口打开
    hCom_ = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hCom_ == INVALID_HANDLE_VALUE) {
        return false;
    }

    // 配置串口
    DCB dcb;
    if (!GetCommState((HANDLE)hCom_, &dcb)) {
        CloseHandle((HANDLE)hCom_);
        hCom_ = nullptr;
        return false;
    }

    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fErrorChar = FALSE;
    dcb.fNull = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fAbortOnError = FALSE;

    if (!SetCommState((HANDLE)hCom_, &dcb)) {
        CloseHandle((HANDLE)hCom_);
        hCom_ = nullptr;
        return false;
    }

    // 设置超时
    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;

    if (!SetCommTimeouts((HANDLE)hCom_, &timeouts)) {
        CloseHandle((HANDLE)hCom_);
        hCom_ = nullptr;
        return false;
    }

    // 清空缓冲区
    PurgeComm((HANDLE)hCom_, PURGE_TXCLEAR | PURGE_RXCLEAR);

    is_open_ = true;
    return true;
#else
    // Linux 串口打开
    hCom_ = (void*)open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (hCom_ == (void*)-1) {
        return false;
    }

    // 配置串口
    struct termios options;
    tcgetattr((int)hCom_, &options);

    // 设置波特率
    speed_t speed;
    switch (baudrate) {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        default:
            speed = B9600;
            break;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    if (tcsetattr((int)hCom_, TCSANOW, &options) != 0) {
        close((int)hCom_);
        hCom_ = nullptr;
        return false;
    }

    is_open_ = true;
    return true;
#endif
}

void SerialPort::close() {
    if (is_open_) {
#ifdef _WIN32
        CloseHandle((HANDLE)hCom_);
#else
        close((int)hCom_);
#endif
        hCom_ = nullptr;
        is_open_ = false;
    }
}

bool SerialPort::isOpen() const {
    return is_open_;
}

int SerialPort::read(char* buffer, int size) {
    if (!is_open_) {
        return -1;
    }

#ifdef _WIN32
    DWORD bytes_read;
    if (!ReadFile((HANDLE)hCom_, buffer, size, &bytes_read, NULL)) {
        return -1;
    }
    return bytes_read;
#else
    return read((int)hCom_, buffer, size);
#endif
}

int SerialPort::write(const char* buffer, int size) {
    if (!is_open_) {
        return -1;
    }

#ifdef _WIN32
    DWORD bytes_written;
    if (!WriteFile((HANDLE)hCom_, buffer, size, &bytes_written, NULL)) {
        return -1;
    }
    return bytes_written;
#else
    return write((int)hCom_, buffer, size);
#endif
}
