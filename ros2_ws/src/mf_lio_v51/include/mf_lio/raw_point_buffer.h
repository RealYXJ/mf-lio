#pragma once
#include <deque>
#include <mutex>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct StampedPoint {
    double timestamp;
    pcl::PointXYZINormal point;
};

class RawPointBuffer {
public:
    void push(double t, const pcl::PointXYZINormal& pt) {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_.push_back({t, pt});
    }

    // Appends a whole scan atomically. A concurrent reader taking mtx_ via
    // get_window()/latest_time() therefore observes the buffer either fully
    // before or fully after this scan — never mid-scan. Per-point push() left
    // a window where latest_time() could return an intermediate point's
    // timestamp while the rest of the scan was still being inserted, which let
    // the estimation-loop trigger fire against a partially-arrived scan; the
    // set of points captured then depended on real scheduling timing between
    // the pushing thread and the polling thread, not on any message content.
    void push_batch(const std::vector<StampedPoint>& pts) {
        if (pts.empty()) return;
        std::lock_guard<std::mutex> lk(mtx_);
        buf_.insert(buf_.end(), pts.begin(), pts.end());
    }

    std::vector<StampedPoint> get_window(double t_start, double t_end) const {
        std::lock_guard<std::mutex> lk(mtx_);
        std::vector<StampedPoint> out;
        for (const auto& sp : buf_)
            if (sp.timestamp >= t_start && sp.timestamp <= t_end)
                out.push_back(sp);
        return out;
    }

    std::vector<StampedPoint> get_latest_ms(double duration_ms) const {
        double t_end;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (buf_.empty()) return {};
            t_end = buf_.back().timestamp;
        }
        double t_start = t_end - duration_ms * 1e-3;
        return get_window(t_start, t_end);
    }

    void clear_before(double t) {
        std::lock_guard<std::mutex> lk(mtx_);
        while (!buf_.empty() && buf_.front().timestamp < t)
            buf_.pop_front();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return buf_.empty();
    }

    double latest_time() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return buf_.empty() ? 0.0 : buf_.back().timestamp;
    }

private:
    mutable std::mutex mtx_;
    std::deque<StampedPoint> buf_;
};
