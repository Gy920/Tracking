#ifndef BATCHTRAKCER_HPP_
#define BATCHTRACKER_HPP_

#include <opencv4/opencv2/tracking.hpp>
#include <opencv4/opencv2/opencv.hpp>
namespace Track
{

    class BatchTracker
    {
    public:
        BatchTracker(cv::Rect2d &rect);

        cv::Rect2d predict(const double d_t = 1);

        cv::Rect2d correct(cv::Rect2d &rect);

        cv::Rect2d getLastRect();

    private:
        void kfInit(cv::Rect2d &rect);

        cv::Rect2d getRect(float cx, float cy, float s, float r);

        cv::KalmanFilter kf;

        cv::Rect2d last_rect;

    public:
        // param
        int since_update_age = 0;
        static int tracker_count;
        int id;
        int m_hits;
        int m_hit_streak;
        int m_age;
    };

}
#endif