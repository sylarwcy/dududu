#include "AutomaticStateMachine.h"

AutomaticStateMachine::AutomaticStateMachine()
    : current_status_(SteelPlateStatus::NO_PLATE)
    , is_running_(false)
    , stable_detection_count_(0)
    , tracking_state_(TrackingState::NORMAL) {
    resetTracking();
}

AutomaticStateMachine::~AutomaticStateMachine() {
    stop();
}

void AutomaticStateMachine::start() {
    is_running_ = true;
    std::cout << "钢板跟踪系统启动" << std::endl;
}

void AutomaticStateMachine::stop() {
    is_running_ = false;
    std::cout << "钢板跟踪系统停止" << std::endl;
}

void AutomaticStateMachine::updateDetection(const DetectionResult& detection) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 保存最近的检测结果
    recent_detections_.push_back(detection);
    if (recent_detections_.size() > 10) {
        recent_detections_.erase(recent_detections_.begin());
    }

    // 更新位置历史（只在有钢板时）
    if (detection.has_steel_plate) {
        updatePositionHistory(detection);
    }

    processDetection(detection);
}

void AutomaticStateMachine::updateMotorSignals(const MotorSignals& signal) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    processMotorSignals(signal);
}

void AutomaticStateMachine::resetTracking() {
    current_tracking_.plate_number = "";
    current_tracking_.status = SteelPlateStatus::NO_PLATE;
    current_tracking_.enter_direction = EnterDirection::UNKNOWN;
    current_tracking_.leave_direction = LeaveDirection::UNKNOWN;
    current_tracking_.is_tracking = false;
    stable_detection_count_ = 0;

    // 确保回到正常状态
    tracking_state_ = TrackingState::NORMAL;
    pending_leave_direction_ = LeaveDirection::UNKNOWN;

    // 清空位置历史
    clearPositionHistory();
}

void AutomaticStateMachine::processDetection(const DetectionResult& detection) {
    // 先检查是否在等待离开确认状态或缓冲期
    if (tracking_state_ == TrackingState::WAITING_FOR_LEAVE_CONFIRM) {
        checkLeaveConfirmation(detection);
        return;
    } else if (tracking_state_ == TrackingState::BUFFER_PERIOD) {
        checkBufferPeriod(detection);
        return;
    }

    // 原有的状态处理逻辑
    switch (current_status_) {
        case SteelPlateStatus::NO_PLATE:
            handleNoPlateState(detection);
            break;
        case SteelPlateStatus::PLATE_DETECTED:
            handlePlateDetectedState(detection);
            break;
        case SteelPlateStatus::PLATE_IDENTIFIED:
            handlePlateIdentifiedState(detection);
            break;
    }
}

void AutomaticStateMachine::handleNoPlateState(const DetectionResult& detection) {
    if (detection.has_steel_plate && !detection.has_crane) {
        // 检测到钢板且没有天车干扰，开始跟踪
        stable_detection_count_++;

        if (stable_detection_count_ >= STABLE_DETECTION_COUNT_THRESHOLD) {
            startPlateTracking(detection);
        }
    } else {
        stable_detection_count_ = 0;
    }
}

void AutomaticStateMachine::handlePlateDetectedState(const DetectionResult& detection) {
    if (!detection.has_steel_plate) {
        // 钢板消失
        handlePlateDisappearance();
        return;
    }

    if (detection.has_crane) {
        // 天车出现，可能在进行吊运操作
        handleCraneOperation(detection);
        return;
    }

    // 尝试识别钢板号码
    if (!detection.plate_number.empty() &&
        detection.confidence >= PLATE_NUMBER_CONFIDENCE_THRESHOLD) {
        identifyPlateNumber(detection.plate_number);
    }
}

void AutomaticStateMachine::handlePlateIdentifiedState(const DetectionResult& detection) {
    if (!detection.has_steel_plate) {
        // 钢板消失
        handlePlateDisappearance();
        return;
    }

    if (detection.has_crane) {
        // 天车出现，可能在进行吊运操作
        handleCraneOperation(detection);
    }
}

void AutomaticStateMachine::processMotorSignals(const MotorSignals& signal) {
    if (!current_tracking_.is_tracking) {
        // 不在跟踪状态，检查是否可以开始跟踪（通过电机信号判断钢板进入）
        checkPlateEntryByMotor(signal);
    } else {
        // 在跟踪状态，检查钢板离开
        checkPlateLeaveByMotor(signal);
    }
}

void AutomaticStateMachine::checkPlateEntryByMotor(const MotorSignals& signal) {
    if (signal.tilter_direction == Direction::FORWARD) {
        // 翻板机正向转动，将钢板从辊道翻到翻板台
        std::cout << "检测到翻板机正向转动，预计钢板将从辊道翻到翻板台" << std::endl;
    } else if (signal.transfer_direction == Direction::FORWARD) {
        // 移钢机正向转动，将钢板从辊道移到翻板台
        std::cout << "检测到移钢机正向转动，预计钢板将从辊道移到翻板台" << std::endl;
    } else if (signal.transfer_direction == Direction::REVERSE) {
        // 移钢机反向转动，将钢板从远处移到翻板台
        std::cout << "检测到移钢机反向转动，预计钢板将从远处移到翻板台" << std::endl;
    }
}

void AutomaticStateMachine::checkPlateLeaveByMotor(const MotorSignals& signal) {
    // 如果已经在等待确认状态，忽略新的电机信号（避免重复触发）
    if (tracking_state_ == TrackingState::WAITING_FOR_LEAVE_CONFIRM ||
        tracking_state_ == TrackingState::BUFFER_PERIOD) {
        return;
    }

    LeaveDirection detected_direction = LeaveDirection::UNKNOWN;

    if (signal.tilter_direction == Direction::REVERSE) {
        // 翻板机反向转动，将钢板从翻板台翻到辊道
        std::cout << "检测到翻板机反向转动，启动离开确认等待" << std::endl;
        detected_direction = LeaveDirection::TILTER_TO_ROLLER;
    } else if (signal.transfer_direction == Direction::REVERSE) {
        // 移钢机反向转动，将钢板从翻板台移到辊道
        std::cout << "检测到移钢机反向转动，启动离开确认等待" << std::endl;
        detected_direction = LeaveDirection::TRANSFER_MACHINE_TO_ROLLER;
    } else if (signal.transfer_direction == Direction::FORWARD) {
        // 移钢机正向转动，将钢板从翻板台移到远处
        std::cout << "检测到移钢机正向转动，启动离开确认等待" << std::endl;
        detected_direction = LeaveDirection::TRANSFER_MACHINE_TO_FAR;
    }

    // 如果检测到有效的离开信号，启动离开确认等待
    if (detected_direction != LeaveDirection::UNKNOWN && current_tracking_.is_tracking) {
        startLeaveConfirmation(detected_direction);
    }
}

void AutomaticStateMachine::startLeaveConfirmation(LeaveDirection direction) {
    tracking_state_ = TrackingState::WAITING_FOR_LEAVE_CONFIRM;
    pending_leave_direction_ = direction;
    wait_start_time_ = std::chrono::system_clock::now();

    std::cout << "开始离开确认等待 - 方向: " << getLeaveDirectionString(direction)
              << " 开始时间: " << timeToString(wait_start_time_) << std::endl;
    std::cout << "等待视觉检测确认钢板消失..." << std::endl;
}

void AutomaticStateMachine::checkLeaveConfirmation(const DetectionResult& detection) {
    // 检查超时
    if (isLeaveConfirmationTimeout()) {
        std::cout << "离开确认超时，进入缓冲期" << std::endl;
        tracking_state_ = TrackingState::BUFFER_PERIOD;
        return;
    }

    // 检查钢板是否消失
    if (!detection.has_steel_plate) {
        // 钢板消失，确认离开
        std::cout << "视觉检测确认钢板已离开，完成跟踪" << std::endl;
        endPlateTracking(pending_leave_direction_);
        tracking_state_ = TrackingState::NORMAL;
    } else {
        // 钢板仍然存在，继续等待
        auto current_time = std::chrono::system_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - wait_start_time_).count();
        int remaining_ms = LEAVE_CONFIRM_TIMEOUT_MS - elapsed_ms;

        std::cout << "视觉检测钢板仍在翻板台上，继续等待确认... 剩余时间: "
                  << remaining_ms << "ms" << std::endl;
    }
}

void AutomaticStateMachine::checkBufferPeriod(const DetectionResult& detection) {
    // 检查缓冲期超时
    if (isBufferPeriodTimeout()) {
        std::cout << "缓冲期超时，钢板可能未离开，取消离开确认" << std::endl;
        cancelLeaveConfirmation();
        return;
    }

    // 检查钢板是否消失
    if (!detection.has_steel_plate) {
        // 在缓冲期内钢板消失，仍然使用电机信号的方向
        std::cout << "缓冲期内视觉检测确认钢板已离开，完成跟踪" << std::endl;
        endPlateTracking(pending_leave_direction_);
        tracking_state_ = TrackingState::NORMAL;
    } else {
        // 钢板仍然存在，继续缓冲期等待
        auto current_time = std::chrono::system_clock::now();
        auto total_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - wait_start_time_).count();
        auto buffer_elapsed_ms = total_elapsed_ms - LEAVE_CONFIRM_TIMEOUT_MS;
        int remaining_ms = BUFFER_PERIOD_MS - buffer_elapsed_ms;

        std::cout << "缓冲期内视觉检测钢板仍在，继续等待... 缓冲期剩余: "
                  << remaining_ms << "ms" << std::endl;
    }
}

bool AutomaticStateMachine::isLeaveConfirmationTimeout() const {
    auto current_time = std::chrono::system_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - wait_start_time_).count();
    return elapsed_ms > LEAVE_CONFIRM_TIMEOUT_MS;
}

bool AutomaticStateMachine::isBufferPeriodTimeout() const {
    auto current_time = std::chrono::system_clock::now();
    auto total_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - wait_start_time_).count();
    return total_elapsed_ms > (LEAVE_CONFIRM_TIMEOUT_MS + BUFFER_PERIOD_MS);
}

void AutomaticStateMachine::cancelLeaveConfirmation() {
    tracking_state_ = TrackingState::NORMAL;
    pending_leave_direction_ = LeaveDirection::UNKNOWN;

    std::cout << "离开确认已取消，回到正常跟踪状态" << std::endl;
}

void AutomaticStateMachine::startPlateTracking(const DetectionResult& detection) {
    current_tracking_.is_tracking = true;
    current_tracking_.enter_time = detection.timestamp;
    current_tracking_.status = SteelPlateStatus::PLATE_DETECTED;
    current_status_ = SteelPlateStatus::PLATE_DETECTED;

    // 清空之前的位置历史
    clearPositionHistory();

    // 记录初始位置
    if (detection.has_steel_plate) {
        updatePositionHistory(detection);
    }

    // 确定进入方向
    current_tracking_.enter_direction = determineEnterDirection();

    std::cout << "开始钢板跟踪 - 进入方向: " << getEnterDirectionString(current_tracking_.enter_direction)
              << " 时间: " << timeToString(detection.timestamp) << std::endl;

    reportPlateOnTilter();
}

void AutomaticStateMachine::endPlateTracking(LeaveDirection direction) {
    if (!current_tracking_.is_tracking) return;

    current_tracking_.leave_direction = direction;
    current_tracking_.leave_time = std::chrono::system_clock::now();
    current_tracking_.is_tracking = false;

    std::cout << "结束钢板跟踪 - 钢板号码: " <<
        (current_tracking_.plate_number.empty() ? "未知" : current_tracking_.plate_number)
              << " 离开方向: " << getLeaveDirectionString(direction)
              << " 时间: " << timeToString(current_tracking_.leave_time) << std::endl;

    reportPlateLeave(direction);

    resetTracking();
    current_status_ = SteelPlateStatus::NO_PLATE;
}

void AutomaticStateMachine::handleCraneOperation(const DetectionResult& detection) {
    if (detection.crane_is_landed && detection.has_steel_plate) {
        // 天车落地且有钢板，可能是放置操作
        if (current_status_ == SteelPlateStatus::NO_PLATE) {
            // 天车放置钢板
            startPlateTracking(detection);
            current_tracking_.enter_direction = EnterDirection::CRANE_PLACE;
        }
    } else if (!detection.has_steel_plate && detection.has_crane && detection.crane_is_landed) {
        // 有天车落地但无钢板，可能是吊走操作
        if (current_tracking_.is_tracking) {
            endPlateTracking(LeaveDirection::CRANE_TAKE_AWAY);
        }
    }
}

void AutomaticStateMachine::identifyPlateNumber(const std::string& plate_number) {
    current_tracking_.plate_number = plate_number;
    current_tracking_.status = SteelPlateStatus::PLATE_IDENTIFIED;
    current_status_ = SteelPlateStatus::PLATE_IDENTIFIED;

    std::cout << "钢板号码识别成功: " << plate_number
              << " 时间: " << timeToString(std::chrono::system_clock::now()) << std::endl;

    reportPlateNumber(plate_number);
}

EnterDirection AutomaticStateMachine::determineEnterDirection() {
    // 这里可以根据最近的电机信号和历史数据来确定进入方向
    // 简化实现，返回未知方向，实际应用中需要更复杂的逻辑
    return EnterDirection::UNKNOWN;
}

// 新增：处理钢板消失
void AutomaticStateMachine::handlePlateDisappearance() {
    // 检查是否天车吊走
    if (!recent_detections_.empty() &&
        recent_detections_.back().has_crane &&
        recent_detections_.back().crane_is_landed) {
        endPlateTracking(LeaveDirection::CRANE_TAKE_AWAY);
    } else {
        // 通过视觉分析移动方向
        LeaveDirection visual_direction = analyzeMovementDirection();
        endPlateTracking(visual_direction);
    }
}

// 新增：分析移动方向
LeaveDirection AutomaticStateMachine::analyzeMovementDirection() {
    if (plate_position_history_.empty()) {
        std::cout << "没有位置历史，无法判断移动方向" << std::endl;
        return LeaveDirection::UNKNOWN;
    }

    // 如果有足够的历史数据，使用移动趋势分析
    if (plate_position_history_.size() >= MIN_MOVEMENT_FRAMES) {
        // 计算平均移动方向
        double total_movement = 0.0;
        int valid_pairs = 0;

        for (size_t i = 1; i < plate_position_history_.size(); ++i) {
            double movement = plate_position_history_[i].first - plate_position_history_[i-1].first;
            total_movement += movement;
            valid_pairs++;
        }

        if (valid_pairs > 0) {
            double avg_movement = total_movement / valid_pairs;
            std::cout << "平均移动量: " << avg_movement << " (正数=向下, 负数=向上)" << std::endl;

            // 判断移动方向
            if (avg_movement > MOVEMENT_THRESHOLD) {
                std::cout << "视觉判断：钢板向下移动，往移钢机远处方向消失" << std::endl;
                return LeaveDirection::TO_FAR_DIRECTION;
            } else if (avg_movement < -MOVEMENT_THRESHOLD) {
                std::cout << "视觉判断：钢板向上移动，往辊道方向消失" << std::endl;
                return LeaveDirection::TO_ROLLER_DIRECTION;
            }
            // 如果移动量太小，继续使用单点位置判断
        }
    }

    // 历史数据不足或移动趋势不明显时，使用单点位置判断
    return analyzeDirectionByLastPosition();
}

// 新增：通过最后位置判断方向
LeaveDirection AutomaticStateMachine::analyzeDirectionByLastPosition() {
    if (plate_position_history_.empty()) {
        return LeaveDirection::UNKNOWN;
    }

    double last_position = plate_position_history_.back().first;
    std::cout << "使用最后位置判断方向，Y坐标: " << last_position
              << " (0=顶部辊道方向, 1=底部远处方向)" << std::endl;

    // 定义画面区域阈值
    const double UPPER_THRESHOLD = 0.3;    // 上半部分：0.0 - 0.3
    const double LOWER_THRESHOLD = 0.7;    // 下半部分：0.7 - 1.0

    if (last_position <= UPPER_THRESHOLD) {
        std::cout << "最后位置在画面上半部分，判断为往辊道方向消失" << std::endl;
        return LeaveDirection::TO_ROLLER_DIRECTION;
    } else if (last_position >= LOWER_THRESHOLD) {
        std::cout << "最后位置在画面下半部分，判断为往移钢机远处方向消失" << std::endl;
        return LeaveDirection::TO_FAR_DIRECTION;
    } else {
        std::cout << "最后位置在画面中间区域，无法确定方向" << std::endl;
        return LeaveDirection::UNKNOWN;
    }
}

// 新增：更新位置历史
void AutomaticStateMachine::updatePositionHistory(const DetectionResult& detection) {
    if (!detection.has_steel_plate) {
        return;
    }

    plate_position_history_.push_back(
        std::make_pair(detection.plate_center_y, detection.timestamp));

    // 保持历史记录数量
    if (plate_position_history_.size() > POSITION_HISTORY_SIZE) {
        plate_position_history_.pop_front();
    }
}

// 新增：清空位置历史
void AutomaticStateMachine::clearPositionHistory() {
    plate_position_history_.clear();
}

void AutomaticStateMachine::reportPlateOnTilter() {
    std::cout << "[上报] 翻板台上有钢板" << std::endl;
    // 这里可以添加实际上报逻辑，如网络请求、消息队列等
}

void AutomaticStateMachine::reportPlateNumber(const std::string& plate_number) {
    std::cout << "[上报] 钢板号码: " << plate_number << std::endl;
    // 这里可以添加实际上报逻辑
}

void AutomaticStateMachine::reportPlateLeave(LeaveDirection direction) {
    std::cout << "[上报] 钢板离开 - 方向: " << getLeaveDirectionString(direction) << std::endl;
    // 这里可以添加实际上报逻辑
}

std::string AutomaticStateMachine::getEnterDirectionString(EnterDirection direction) {
    switch (direction) {
        case EnterDirection::TRANSFER_MACHINE_FROM_ROLLER: return "辊道通过移钢机移来";
        case EnterDirection::TILTER_FROM_ROLLER: return "辊道通过翻板机翻来";
        case EnterDirection::CRANE_PLACE: return "天车吊来放置";
        case EnterDirection::TRANSFER_MACHINE_FROM_FAR: return "移钢机远处反向移来";
        default: return "未知";
    }
}

std::string AutomaticStateMachine::getLeaveDirectionString(LeaveDirection direction) {
    switch (direction) {
        case LeaveDirection::CRANE_TAKE_AWAY: return "天车吊走";
        case LeaveDirection::TRANSFER_MACHINE_TO_ROLLER: return "移钢机移到辊道";
        case LeaveDirection::TILTER_TO_ROLLER: return "翻板机翻到辊道";
        case LeaveDirection::TRANSFER_MACHINE_TO_FAR: return "移钢机移到远处";
        case LeaveDirection::TO_ROLLER_DIRECTION: return "往辊道方向消失";
        case LeaveDirection::TO_FAR_DIRECTION: return "往移钢机远处方向消失";
        default: return "未知";
    }
}

std::string AutomaticStateMachine::timeToString(const std::chrono::system_clock::time_point& time) {
    auto time_t = std::chrono::system_clock::to_time_t(time);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%H:%M:%S", std::localtime(&time_t));
    return std::string(buffer);
}