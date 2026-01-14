/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
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

#ifndef GI_ENGINE_H
#define GI_ENGINE_H

#include <Eigen/Dense>
#include <functional>
#include <vector>

#include "common/types.h"

#include "kf_gins_types.h"

class GIEngine {

public:
    // 故障类型枚举
    // fault type enum
    enum FaultType {
        FAULT_NONE = 0,
        FAULT_IMU_LOST,
        FAULT_GNSS_LOST
    };

    // 故障级别枚举
    // fault level enum
    enum FaultLevel {
        LEVEL_INFO = 0,
        LEVEL_WARNING,
        LEVEL_ERROR,
        LEVEL_FATAL
    };

    explicit GIEngine(GINSOptions &options);

    ~GIEngine() = default;

    // 回调函数类型
    // callback function type
    using NavResultCallback = std::function<void(double, const NavState&, const Eigen::MatrixXd&)>;

    // 设置回调函数
    // set callback function
    void setNavResultCallback(NavResultCallback callback) {
        nav_result_callback_ = callback;
    }

    /**
     * @brief 添加新的IMU数据，(不)补偿IMU误差
     *        add new imudata, do (not) compensate imu error
     * @param [in] imu        新的IMU原始数据
     *                        new raw imudata
     * @param [in] compensate 是否补偿IMU误差
     *                        if compensate imu error to new imudata
     * */
    void addImuData(const IMU &imu, bool compensate = false) {

        imupre_ = imucur_;
        imucur_ = imu;

        if (compensate) {
            imuCompensate(imucur_);
        }
        
        // 检查IMU信号丢失
        // check IMU signal loss
        if (last_imu_time_ > 0) {
            double imu_time_diff = imucur_.time - last_imu_time_;
            if (imu_time_diff > IMU_TIMEOUT_THRESHOLD) {
                imu_lost_ = true;
                char buffer[128];
                sprintf_s(buffer, sizeof(buffer), "IMU signal lost at %.3f! Time diff: %.3fs", timestamp_, imu_time_diff);
                reportFault(FAULT_IMU_LOST, LEVEL_ERROR, buffer);
            } else if (imu_lost_) {
                imu_lost_ = false;
                reportFault(FAULT_IMU_LOST, LEVEL_INFO, "IMU signal recovered");
            }
        }
        
        last_imu_time_ = imucur_.time;
    }

    /**
     * @brief 添加新的GNSS数据
     *        add new gnssdata
     * @param [in] gnss 新的GNSS数据
     *                  new gnssdata
     * */
    void addGnssData(const GNSS &gnss) {

        gnssdata_ = gnss;
        // 暂不进行数据有效性检查，GNSS数据默认有效
        // do not check the validity of gnssdata, the gnssdata is valid by default
        gnssdata_.isvalid = true;
        
        // 更新最后一次有效GNSS时间
        // update last valid GNSS time
        gnss_last_valid_time_ = gnss.time;
        
        // 如果之前GNSS信号丢失，重置标志
        // reset GNSS lost flag if it was lost before
        if (gnss_lost_) {
            gnss_lost_ = false;
            reportFault(FAULT_GNSS_LOST, LEVEL_INFO, "GNSS signal recovered");
        }
        
        gnss_lost_time_ = 0.0;
    }

    /**
     * @brief 处理新的IMU数据
     *        process new imudata
     * */
    void newImuProcess();

    /**
     * @brief 内插增量形式的IMU数据到指定时刻
     *        interpolate incremental imudata to given timestamp
     * @param [in]     imu1      前一时刻IMU数据
     *                           the previous imudata
     * @param [in,out] imu2      当前时刻IMU数据
     *                           the current imudata
     * @param [in]     timestamp 给定内插到的时刻
     *                           given interpolate timestamp
     * @param [in,out] midimu    输出内插时刻的IMU数据
     *                           output imudata at given timestamp
     * */
    static void imuInterpolate(const IMU &imu1, IMU &imu2, const double timestamp, IMU &midimu) {

        if (imu1.time > timestamp || imu2.time < timestamp) {
            return;
        }

        double lamda = (timestamp - imu1.time) / (imu2.time - imu1.time);

        midimu.time   = timestamp;
        midimu.dtheta = imu2.dtheta * lamda;
        midimu.dvel   = imu2.dvel * lamda;
        midimu.dt     = timestamp - imu1.time;

        imu2.dtheta = imu2.dtheta - midimu.dtheta;
        imu2.dvel   = imu2.dvel - midimu.dvel;
        imu2.dt     = imu2.dt - midimu.dt;
    }

    /**
     * @brief 获取当前时间
     *        get current time
     * */
    double timestamp() const {
        return timestamp_;
    }

    /**
     * @brief 获取当前IMU状态
     *        get current navigation state
     * */
    NavState getNavState();

    /**
     * @brief 获取当前状态协方差
     *        get current state covariance
     * */
    Eigen::MatrixXd getCovariance() {
        return Cov_;
    }
    
    /**
     * @brief 检查IMU信号状态
     *        check IMU signal status
     * */
    void checkImuSignal();
    
    /**
     * @brief 检查GNSS信号状态
     *        check GNSS signal status
     * */
    void checkGnssSignal();
    
    /**
     * @brief 报告故障信息
     *        report fault information
     * @param [in] fault_type 故障类型
     *                        fault type
     * @param [in] fault_level 故障级别
     *                        fault level
     * @param [in] message 故障信息
     *                     fault message
     * */
    void reportFault(FaultType fault_type, FaultLevel fault_level, const std::string &message);
    
    /**
     * @brief 获取当前故障状态
     *        get current fault status
     * @return 当前故障类型
     *         current fault type
     * */
    FaultType getFaultStatus() const {
        if (imu_lost_) return FAULT_IMU_LOST;
        if (gnss_lost_) return FAULT_GNSS_LOST;
        return FAULT_NONE;
    }

private:
    /**
     * @brief 初始化系统状态和协方差
     *        initialize state and state covariance
     * @param [in] initstate     初始状态
     *                           initial state
     * @param [in] initstate_std 初始状态标准差
     *                           initial state std
     * */
    void initialize(const NavState &initstate, const NavState &initstate_std);

    /**
     * @brief 当前IMU误差补偿到IMU数据中
     *        componsate imu error to the imudata
     * @param [in,out] imu 需要补偿的IMU数据
     *                     imudata to be compensated
     * */
    void imuCompensate(IMU &imu);

    /**
     * @brief 判断是否需要更新,以及更新哪一时刻系统状态
     *        determine if we should do upate and which navstate to update
     * @param [in] imutime1   上一IMU状态时间
     *                        the last state time
     * @param [in] imutime2   当前IMU状态时间
     *                        the current state time
     * @param [in] updatetime 状态更新的时间
     *                        time to update state
     * @return 0: 不需要更新
     *            donot need update
     *         1: 需要更新上一IMU状态
     *            update the last navstate
     *         2: 需要更新当前IMU状态
     *            update the current navstate
     *         3: 需要将IMU进行内插到状态更新时间
     *            need interpolate imudata to updatetime
     * */
    int isToUpdate(double imutime1, double imutime2, double updatetime) const;

    /**
     * @brief 进行INS状态更新(IMU机械编排算法), 并计算IMU状态转移矩阵和噪声阵
     *        do INS state update(INS mechanization), and compute state transition matrix and noise matrix
     * @param [in,out] imupre 前一时刻IMU数据
     *                        imudata at the previous epoch
     * @param [in,out] imucur 当前时刻IMU数据
     *                        imudata at the current epoch
     * */
    void insPropagation(IMU &imupre, IMU &imucur);

    /**
     * @brief 使用GNSS位置观测更新系统状态
     *        update state using gnss position
     * @param [in,out] gnssdata
     * */
    void gnssUpdate(GNSS &gnssdata);

    /**
     * @brief Kalman 预测,
     *        Kalman Filter Predict process
     * @param [in,out] Phi 状态转移矩阵
     *                     state transition matrix
     * @param [in,out] Qd  传播噪声矩阵
     *                     propagation noise matrix
     * */
    void EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd);

    /**
     * @brief Kalman 更新
     *        Kalman Filter Update process
     * @param [in] dz 观测新息
     *                measurement innovation
     * @param [in] H  观测矩阵
     *                measurement matrix
     * @param [in] R  观测噪声阵
     *                measurement noise matrix
     * */
    void EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R);

    /**
     * @brief 反馈误差状态到当前状态
     *        feedback error state to the current state
     * */
    void stateFeedback();

    /**
     * @brief 检查协方差对角线元素是否都为正
     *        Check if covariance diagonal elements are all positive
     * */
    void checkCov() {

        for (int i = 0; i < RANK; i++) {
            if (Cov_(i, i) < 0) {
                std::cout << "Covariance is negative at " << std::setprecision(10) << timestamp_ << " !" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }

private:
    GINSOptions options_;

    double timestamp_;

    // 更新时间对齐误差，IMU状态和观测信息误差小于它则认为两者对齐
    // updata time align error
    const double TIME_ALIGN_ERR = 0.001;
    
    // IMU超时阈值（秒）
    // IMU timeout threshold (seconds)
    const double IMU_TIMEOUT_THRESHOLD = 1.0;
    
    // GNSS超时阈值（秒）
    // GNSS timeout threshold (seconds)
    const double GNSS_TIMEOUT_THRESHOLD = 5.0;

    // IMU和GNSS原始数据
    // raw imudata and gnssdata
    IMU imupre_;
    IMU imucur_;
    GNSS gnssdata_;

    // IMU状态（位置、速度、姿态和IMU误差）
    // imu state (position, velocity, attitude and imu error)
    PVA pvacur_;
    PVA pvapre_;
    ImuError imuerror_;

    // Kalman滤波相关
    // ekf variables
    Eigen::MatrixXd Cov_;
    Eigen::MatrixXd Qc_;
    Eigen::MatrixXd dx_;
    
    // 预分配矩阵，避免重复内存操作
    // Pre-allocated matrices to avoid repeated memory operations
    Eigen::MatrixXd Phi_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Qd_;
    Eigen::MatrixXd G_;

    // 导航结果回调函数
    // navigation result callback function
    NavResultCallback nav_result_callback_;

    const int RANK      = 21;
    const int NOISERANK = 18;

    // 状态ID和噪声ID
    // state ID and noise ID
    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, BG_ID = 9, BA_ID = 12, SG_ID = 15, SA_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, BGSTD_ID = 6, BASTD_ID = 9, SGSTD_ID = 12, SASTD_ID = 15 };
    
    // 信号丢失检测相关
    // signal loss detection related
    bool imu_lost_;                    // IMU信号丢失标志
    bool gnss_lost_;                   // GNSS信号丢失标志
    double gnss_last_valid_time_;      // 最后一次有效GNSS时间
    double gnss_lost_time_;            // GNSS信号丢失时长
    double last_imu_time_;             // 上一次IMU数据时间
};

#endif // GI_ENGINE_H
