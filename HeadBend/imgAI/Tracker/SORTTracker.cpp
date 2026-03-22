#include "SORTTracker.h"
#include <algorithm>
#include <set>

SORTTracker::SORTTracker(int max_age, int min_hits, double iouThreshold, int max_trackers)
    : max_age(max_age), min_hits(min_hits), iouThreshold(iouThreshold), max_trackers(max_trackers), frame_count(0) {
    KalmanTracker::kf_count = 0;
}

double SORTTracker::GetIOU(cv::Rect_<int> bb_test, cv::Rect_<int> bb_gt) {
    // 计算交集
    int x1 = std::max(bb_test.x, bb_gt.x);
    int y1 = std::max(bb_test.y, bb_gt.y);
    int x2 = std::min(bb_test.x + bb_test.width, bb_gt.x + bb_gt.width);
    int y2 = std::min(bb_test.y + bb_test.height, bb_gt.y + bb_gt.height);

    int intersection = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int area_test = bb_test.width * bb_test.height;
    int area_gt = bb_gt.width * bb_gt.height;
    int union_area = area_test + area_gt - intersection;

    return union_area > 0 ? (double)intersection / union_area : 0.0;
}

void SORTTracker::updateHistory(int id, const cv::Rect_<int> &box) {
    // 如果是第一次出现，保存为第一帧
    if (first_frame_boxes.find(id) == first_frame_boxes.end()) {
        first_frame_boxes[id] = box;
    }

    // 添加到历史记录（限制长度，保留最近20帧）
    track_history[id].push_back(box);
    if (track_history[id].size() > 20) {
        track_history[id].erase(track_history[id].begin());
    }
}

void SORTTracker::clearHistory(int id) {
    track_history.erase(id);
    first_frame_boxes.erase(id);
}

std::vector<TrackingBox> SORTTracker::update(const std::vector<cv::Rect_<int>> &detections) {
    frame_count++;
    std::vector<TrackingBox> frameTrackingResult;

    // 第一帧初始化
    if (trackers.empty()) {
        for (const auto &det : detections) {
            if (trackers.size() >= max_trackers) break; // 不超过最大跟踪器数量
            KalmanTracker trk(det);
            trackers.push_back(trk);
            updateHistory(trk.m_id + 1, det);
        }
        return frameTrackingResult;
    }

    // 预测
    std::vector<cv::Rect_<int>> predictedBoxes;
    for (auto it = trackers.begin(); it != trackers.end();) {
        cv::Rect_<int> pBox = (*it).predict();
        if (pBox.width > 0 && pBox.height > 0) {  // 更宽松的检查
            predictedBoxes.push_back(pBox);
            it++;
        } else {
            // 移除无效的跟踪器
            int id_to_remove = (*it).m_id + 1;
            it = trackers.erase(it);
            clearHistory(id_to_remove);
        }
    }

    // 贪婪IOU匹配（适合钢板场景）
    std::vector<bool> matchedDetections(detections.size(), false);
    std::vector<bool> matchedTrackers(predictedBoxes.size(), false);

    // 对每个跟踪器寻找最佳检测匹配
    for (size_t i = 0; i < predictedBoxes.size(); i++) {
        if (matchedTrackers[i]) continue;

        double bestIOU = 0;
        int bestDetectionIdx = -1;

        for (size_t j = 0; j < detections.size(); j++) {
            if (matchedDetections[j]) continue;

            double iou = GetIOU(predictedBoxes[i], detections[j]);
            if (iou > iouThreshold && iou > bestIOU) {
                bestIOU = iou;
                bestDetectionIdx = j;
            }
        }

        if (bestDetectionIdx != -1) {
            trackers[i].update(detections[bestDetectionIdx]);
            matchedDetections[bestDetectionIdx] = true;
            matchedTrackers[i] = true;
            updateHistory(trackers[i].m_id + 1, detections[bestDetectionIdx]);
        }
    }

    // 为未匹配的检测创建新跟踪器（不超过最大数量）
    for (size_t j = 0; j < detections.size(); j++) {
        if (!matchedDetections[j] && trackers.size() < max_trackers) {
            KalmanTracker tracker(detections[j]);
            trackers.push_back(tracker);
            updateHistory(tracker.m_id + 1, detections[j]);
        }
    }

    // 清理丢失的跟踪器
    auto it = trackers.begin();
    while (it != trackers.end()) {
        if (it->m_time_since_update > max_age) {
            int id_to_remove = it->m_id + 1;
            it = trackers.erase(it);
            clearHistory(id_to_remove);
        } else {
            it++;
        }
    }

    // 输出跟踪结果
    for (auto &tracker : trackers) {
        if (tracker.m_time_since_update < 1 &&  // 当前帧有更新
            (tracker.m_hit_streak >= min_hits || frame_count <= min_hits)) {  // 满足最小命中次数或是前几帧
            TrackingBox res;
            res.box = tracker.get_state();
            res.id = tracker.m_id + 1;
            res.frame = frame_count;
            frameTrackingResult.push_back(res);
        }
    }

    return frameTrackingResult;
}

void SORTTracker::reset() {
    trackers.clear();
    track_history.clear();
    first_frame_boxes.clear();
    frame_count = 0;
    KalmanTracker::kf_count = 0;
}

size_t SORTTracker::getTrackerCount() const {
    return trackers.size();
}

cv::Rect_<int> SORTTracker::getFirstFrameBox(int id) const {
    auto it = first_frame_boxes.find(id);
    return it != first_frame_boxes.end() ? it->second : cv::Rect_<int>();
}

cv::Rect_<int> SORTTracker::getLatestBox(int id) const {
    auto it = track_history.find(id);
    if (it != track_history.end() && !it->second.empty()) {
        return it->second.back();
    }
    return cv::Rect_<int>();
}

std::set<int> SORTTracker::getActiveIDs() const {
    std::set<int> activeIDs;
    for (const auto& tracker : trackers) {
        if (tracker.m_time_since_update < 1) {  // 当前帧活跃的跟踪器
            activeIDs.insert(tracker.m_id + 1);
        }
    }
    return activeIDs;
}

std::vector<cv::Rect_<int>> SORTTracker::getHistory(int id) const {
    auto it = track_history.find(id);
    return it != track_history.end() ? it->second : std::vector<cv::Rect_<int>>();
}

std::map<int, std::vector<cv::Rect_<int>>> SORTTracker::getAllHistory() const {
    return track_history;
}

void SORTTracker::drawTracks(cv::Mat& image) const {
    std::set<int> activeIDs = getActiveIDs();

    // 为每个ID分配固定颜色
    std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 255, 0),    // 绿色
        cv::Scalar(255, 0, 0),    // 蓝色
        cv::Scalar(0, 0, 255),    // 红色
        cv::Scalar(255, 255, 0),  // 青色
        cv::Scalar(255, 0, 255),  // 紫色
        cv::Scalar(0, 255, 255),  // 黄色
        cv::Scalar(128, 128, 128),// 灰色
        cv::Scalar(128, 0, 0),    // 深蓝
        cv::Scalar(0, 128, 0),    // 深绿
        cv::Scalar(0, 0, 128)     // 深红
    };

    int colorIndex = 0;
    for (int id : activeIDs) {
        cv::Rect_<int> box = getLatestBox(id);
        if (box.area() > 0) {
            cv::Scalar color = colors[colorIndex % colors.size()];
            cv::rectangle(image, box, color, 2);
            cv::putText(image, "Steel_" + std::to_string(id),
                        cv::Point(box.x, box.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            colorIndex++;
        }
    }
}

void SORTTracker::drawHistory(cv::Mat& image, int id, const cv::Scalar& color,
                             int thickness, bool drawPoints) const {
    auto it = track_history.find(id);
    if (it == track_history.end()) return;

    const std::vector<cv::Rect_<int>>& history = it->second;

    // 绘制轨迹线
    if (history.size() > 1) {
        for (size_t i = 1; i < history.size(); i++) {
            cv::Point prev(history[i-1].x + history[i-1].width / 2,
                         history[i-1].y + history[i-1].height / 2);
            cv::Point curr(history[i].x + history[i].width / 2,
                         history[i].y + history[i].height / 2);
            cv::line(image, prev, curr, color, thickness);
        }
    }

    // 绘制轨迹点（可选）
    if (drawPoints) {
        for (size_t i = 0; i < history.size(); i++) {
            cv::Point center(history[i].x + history[i].width / 2,
                           history[i].y + history[i].height / 2);
            cv::circle(image, center, 3, color, -1);
        }
    }
}
