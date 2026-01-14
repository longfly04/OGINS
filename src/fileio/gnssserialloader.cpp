#include "fileio/gnssserialloader.h"

#include <iostream>
#include <sstream>
#include <vector>

#include <Eigen/Geometry>

using Eigen::Vector3d;

GnssSerialLoader::GnssSerialLoader(const std::string& port, int baudrate) {
    if (!serial_.open(port, baudrate)) {
        is_open_ = false;
    } else {
        is_open_ = true;
        gnss_.isvalid = false;
    }
}

const GNSS& GnssSerialLoader::next() {
    gnss_.time = 0;
    gnss_.blh = Vector3d::Zero();
    gnss_.std = Vector3d::Zero();
    gnss_.isvalid = false;

    char buffer[1024] = {0};
    int bytes_read = serial_.read(buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        append(buffer, bytes_read);

        size_t start_idx = buffer_.find('$');
        while (start_idx != std::string::npos) {
            size_t end_idx = buffer_.find('\n', start_idx);
            if (end_idx != std::string::npos) {
                std::string nmea = buffer_.substr(start_idx, end_idx - start_idx + 1);
                buffer_ = buffer_.substr(end_idx + 1);
                if (parseNMEA(nmea)) {
                    break;
                }
            } else {
                break;
            }
            start_idx = buffer_.find('$');
        }
    }

    return gnss_;
}

bool GnssSerialLoader::isOpen() const {
    return is_open_;
}

void GnssSerialLoader::append(const char* data, int size) {
    buffer_.append(data, size);
    // 限制缓冲区大小，防止内存溢出
    if (buffer_.size() > 4096) {
        buffer_ = buffer_.substr(buffer_.size() - 4096);
    }
}

bool GnssSerialLoader::parseNMEA(const std::string& nmea) {
    // 检查是否是GGA消息（包含位置信息）
    if (nmea.substr(0, 6) == "$GPGGA" || nmea.substr(0, 6) == "$GNGGA") {
        std::vector<std::string> fields = split(nmea, ',');
        if (fields.size() >= 15) {
            // 解析时间
            std::string time_str = fields[1];
            if (!time_str.empty()) {
                double time = std::stod(time_str) / 10000.0;
                // 注意：这里需要根据实际情况处理GPS周和秒
                gnss_.time = time;
            }
            
            // 解析纬度
            std::string lat_str = fields[2];
            std::string lat_dir = fields[3];
            if (!lat_str.empty() && !lat_dir.empty()) {
                double lat_deg = std::stod(lat_str.substr(0, 2));
                double lat_min = std::stod(lat_str.substr(2));
                double lat = lat_deg + lat_min / 60.0;
                if (lat_dir == "S") {
                    lat = -lat;
                }
                gnss_.blh[0] = lat;
            }
            
            // 解析经度
            std::string lon_str = fields[4];
            std::string lon_dir = fields[5];
            if (!lon_str.empty() && !lon_dir.empty()) {
                double lon_deg = std::stod(lon_str.substr(0, 3));
                double lon_min = std::stod(lon_str.substr(3));
                double lon = lon_deg + lon_min / 60.0;
                if (lon_dir == "W") {
                    lon = -lon;
                }
                gnss_.blh[1] = lon;
            }
            
            // 解析高度
            std::string alt_str = fields[9];
            if (!alt_str.empty()) {
                gnss_.blh[2] = std::stod(alt_str);
            }
            
            // 解析定位质量
            std::string quality_str = fields[6];
            if (!quality_str.empty()) {
                int quality = std::stoi(quality_str);
                gnss_.isvalid = (quality >= 1);
            }
            
            // 设置标准差（根据实际情况调整）
            gnss_.std[0] = 0.05;
            gnss_.std[1] = 0.05;
            gnss_.std[2] = 0.1;
            
            return true;
        }
    }
    
    return false;
}

std::vector<std::string> GnssSerialLoader::split(const std::string& str, char delimiter) {
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
