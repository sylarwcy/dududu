//
// Created by sylar on 25-11-13.
//

#ifndef AUTOMATICSTATEMACHINE_H
#define AUTOMATICSTATEMACHINE_H

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <unordered_map>
#include <deque>

// 钢板状态枚举
enum class SteelPlateStatus {
    NO_PLATE,           // 无钢板
    PLATE_DETECTED,     // 检测到钢板但未识别号码
    PLATE_IDENTIFIED    // 钢板号码已识别
};

// 设备运行方向
enum class Direction {
    UNKNOWN,
    FORWARD,    // 正向
    REVERSE     // 反向
};

// 钢板离开方向
enum class LeaveDirection {
    UNKNOWN,
    CRANE_TAKE_AWAY,            // 天车吊走
    TRANSFER_MACHINE_TO_ROLLER, // 移钢机移到辊道
    TILTER_TO_ROLLER,           // 翻板机翻到辊道
    TRANSFER_MACHINE_TO_FAR,    // 移钢机移到远处
    TO_ROLLER_DIRECTION,        // 往辊道方向消失（视觉判断）
    TO_FAR_DIRECTION           // 往远处方向消失（视觉判断）
};

// 钢板进入方向
enum class EnterDirection {
    UNKNOWN,
    TRANSFER_MACHINE_FROM_ROLLER,   // 辊道通过移钢机移来
    TILTER_FROM_ROLLER,             // 辊道通过翻板机翻来
    CRANE_PLACE,                    // 天车吊来放置
    TRANSFER_MACHINE_FROM_FAR       // 移钢机远处反向移来
};

// 目标检测结果 - 增加位置信息
struct DetectionResult {
    bool has_steel_plate;           // 是否有钢板
    bool has_crane;                 // 是否有天车（空中或落地）
    bool crane_is_landed;           // 天车是否落地
    std::string plate_number;       // 钢板号码
    double confidence;              // 识别置信度
    double plate_center_y;          // 钢板中心Y坐标（0-1，0=画面顶部，1=画面底部）
    double plate_height;            // 钢板高度占画面比例
    std::chrono::system_clock::time_point timestamp; // 检测时间戳
};

// 电机信号
struct MotorSignals {
    Direction tilter_direction;     // 翻板机方向
    Direction transfer_direction;   // 移钢机方向
    std::chrono::system_clock::time_point timestamp; // 信号时间戳
};

// 钢板跟踪信息
struct SteelPlateTracking {
    std::string plate_number;
    SteelPlateStatus status;
    EnterDirection enter_direction;
    LeaveDirection leave_direction;
    std::chrono::system_clock::time_point enter_time;
    std::chrono::system_clock::time_point leave_time;
    bool is_tracking;
};

class AutomaticStateMachine {
public:
    AutomaticStateMachine();
    ~AutomaticStateMachine();

    void start();
    void stop();

    // 更新目标检测结果
    void updateDetection(const DetectionResult& detection);

    // 更新电机信号
    void updateMotorSignals(const MotorSignals& signal);

private:
    SteelPlateTracking current_tracking_;
    SteelPlateStatus current_status_;
    std::mutex data_mutex_;
    std::atomic<bool> is_running_;

    // 配置参数
    const double PLATE_NUMBER_CONFIDENCE_THRESHOLD = 0.8;
    const int STABLE_DETECTION_COUNT_THRESHOLD = 3;
    int stable_detection_count_;

    // 历史检测结果用于稳定性判断和方向分析
    std::vector<DetectionResult> recent_detections_;

    // 视觉方向分析相关
    std::deque<std::pair<double, std::chrono::system_clock::time_point>> plate_position_history_; // 位置历史 (y坐标, 时间)
    const int POSITION_HISTORY_SIZE = 10; // 位置历史记录数量
    const double MOVEMENT_THRESHOLD = 0.05; // 移动判断阈值
    const int MIN_MOVEMENT_FRAMES = 3; // 最小移动帧数

    // 电机触发离开的等待确认机制
    enum class TrackingState {
        NORMAL,                     // 正常跟踪状态
        WAITING_FOR_LEAVE_CONFIRM,  // 等待离开视觉确认状态
        BUFFER_PERIOD               // 超时后的缓冲期
    };

    TrackingState tracking_state_;
    LeaveDirection pending_leave_direction_;       // 待确认的离开方向
    std::chrono::system_clock::time_point wait_start_time_; // 等待开始时间
    const int LEAVE_CONFIRM_TIMEOUT_MS = 5000;     // 离开确认超时时间(5秒)
    const int BUFFER_PERIOD_MS = 2000;             // 缓冲期时间(2秒)

    // 私有方法声明
    void resetTracking();
    void processDetection(const DetectionResult& detection);
    void handleNoPlateState(const DetectionResult& detection);
    void handlePlateDetectedState(const DetectionResult& detection);
    void handlePlateIdentifiedState(const DetectionResult& detection);
    void processMotorSignals(const MotorSignals& signal);
    void checkPlateEntryByMotor(const MotorSignals& signal);
    void checkPlateLeaveByMotor(const MotorSignals& signal);
    void startLeaveConfirmation(LeaveDirection direction);
    void checkLeaveConfirmation(const DetectionResult& detection);
    void checkBufferPeriod(const DetectionResult& detection);
    void cancelLeaveConfirmation();
    bool isLeaveConfirmationTimeout() const;
    bool isBufferPeriodTimeout() const;
    void handlePlateDisappearance(); // 处理钢板消失
    LeaveDirection analyzeMovementDirection(); // 分析移动方向
    LeaveDirection analyzeDirectionByLastPosition(); // 通过最后位置判断方向
    void updatePositionHistory(const DetectionResult& detection); // 更新位置历史
    void clearPositionHistory(); // 清空位置历史
    void startPlateTracking(const DetectionResult& detection);
    void endPlateTracking(LeaveDirection direction);
    void handleCraneOperation(const DetectionResult& detection);
    void identifyPlateNumber(const std::string& plate_number);
    EnterDirection determineEnterDirection();

    // 上报方法
    void reportPlateOnTilter();
    void reportPlateNumber(const std::string& plate_number);
    void reportPlateLeave(LeaveDirection direction);

    // 工具方法
    std::string getEnterDirectionString(EnterDirection direction);
    std::string getLeaveDirectionString(LeaveDirection direction);
    std::string timeToString(const std::chrono::system_clock::time_point& time);


};

#endif //AUTOMATICSTATEMACHINE_H