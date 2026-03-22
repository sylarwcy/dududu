#ifndef SORTTRACKER_H
#define SORTTRACKER_H

#include <vector>
#include <map>
#include <set>
#include "KalmanTracker.h"
#include "opencv2/video/tracking.hpp"

typedef struct TrackingBox {
    int frame;
    int id;
    cv::Rect_<int> box;
} TrackingBox;

class SORTTracker {
private:
    int max_age;
    int min_hits;
    double iouThreshold;
    int max_trackers;  // 最大跟踪器数量

    std::vector<KalmanTracker> trackers;
    int frame_count;

    // 历史记录
    std::map<int, std::vector<cv::Rect_<int>>> track_history;
    std::map<int, cv::Rect_<int>> first_frame_boxes;

    double GetIOU(cv::Rect_<int> bb_test, cv::Rect_<int> bb_gt);
    void updateHistory(int id, const cv::Rect_<int> &box);
    void clearHistory(int id);

public:
    SORTTracker(int max_age = 1, int min_hits = 3, double iouThreshold = 0.5, int max_trackers = 10);

    std::vector<TrackingBox> update(const std::vector<cv::Rect_<int>> &detections);
    void reset();

    // 核心功能
    size_t getTrackerCount() const;
    cv::Rect_<int> getFirstFrameBox(int id) const;
    cv::Rect_<int> getLatestBox(int id) const;
    std::set<int> getActiveIDs() const;

    // 历史记录功能
    std::vector<cv::Rect_<int>> getHistory(int id) const;
    std::map<int, std::vector<cv::Rect_<int>>> getAllHistory() const;

    // 绘制功能
    void drawTracks(cv::Mat& image) const;
    void drawHistory(cv::Mat& image, int id, const cv::Scalar& color = cv::Scalar(0, 255, 0),
                    int thickness = 2, bool drawPoints = true) const;
};

#endif