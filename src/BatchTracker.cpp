#include "../include/BatchTracker.hpp"

namespace Track
{
    int BatchTracker::tracker_count = 0;

    BatchTracker::BatchTracker(cv::Rect2d &rect)
    {
        kfInit(rect);
        last_rect = rect;
        id = tracker_count;
        tracker_count += 1;
    }

    cv::Rect2d BatchTracker::getLastRect()
    {
        return last_rect;
    }

    void BatchTracker::kfInit(cv::Rect2d &rect)
    {
        int stateNum = 7;
        int measureNum = 4;
        kf = cv::KalmanFilter(stateNum, measureNum, 0);
        cv::Mat measurement = cv::Mat::zeros(measureNum, 1, CV_32F);

        kf.transitionMatrix = (cv::Mat_<float>(stateNum, stateNum)
                                   << 1,
                               0, 0, 0, 1, 0, 0,
                               0, 1, 0, 0, 0, 1, 0,
                               0, 0, 1, 0, 0, 0, 1,
                               0, 0, 0, 1, 0, 0, 0,
                               0, 0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 0, 1, 0,
                               0, 0, 0, 0, 0, 0, 1);
        cv::setIdentity(kf.measurementMatrix);
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1));
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1e-2));
        // initialize state vector with bounding box in [cx,cy,s,r] style
        kf.statePost.at<float>(0, 0) = rect.x + rect.width / 2;
        kf.statePost.at<float>(1, 0) = rect.y + rect.height / 2;
        kf.statePost.at<float>(2, 0) = rect.area();
        kf.statePost.at<float>(3, 0) = rect.width / rect.height;
    }

    cv::Rect2d BatchTracker::predict(const double d_t)
    {
        since_update_age += 1;
        m_age += 1;

        if (since_update_age > 0)
            m_hit_streak = 0;

        if (last_rect.height == 0 || last_rect == cv::Rect2d())
            return cv::Rect2d();

        // set
        kf.transitionMatrix = (cv::Mat_<float>(7, 7)
                                   << 1,
                               0, 0, 0, d_t, 0, 0,
                               0, 1, 0, 0, 0, d_t, 0,
                               0, 0, 1, 0, 0, 0, d_t,
                               0, 0, 0, 1, 0, 0, 0,
                               0, 0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 0, 1, 0,
                               0, 0, 0, 0, 0, 0, 1);

        cv::Mat post_state = kf.predict();
        cv::Rect2d predict_box = getRect(post_state.at<float>(0, 0), post_state.at<float>(1, 0), post_state.at<float>(2, 0), post_state.at<float>(3, 0));
        last_rect = predict_box;
        return predict_box;
    }

    cv::Rect2d BatchTracker::getRect(float cx, float cy, float s, float r)
    {
        float w = sqrt(s * r);
        float h = s / w;
        float x = (cx - w / 2);
        float y = (cy - h / 2);

        return (cv::Rect2d(x, y, w, h) & cv::Rect2d(0, 0, w, h));
    }

    cv::Rect2d BatchTracker::correct(cv::Rect2d &rect)
    {
        since_update_age = 0;
        m_hits += 1;
        m_hit_streak += 1;
        if (rect.height == 0 || rect == cv::Rect2d())
            return cv::Rect2d();

        cv::Mat measurement = cv::Mat::zeros(4, 1, CV_32F);
        measurement.at<float>(0, 0) = rect.x + rect.width / 2;
        measurement.at<float>(1, 0) = rect.y + rect.height / 2;
        measurement.at<float>(2, 0) = rect.area();
        measurement.at<float>(3, 0) = rect.width / rect.height;
        cv::Mat post_state = kf.correct(measurement);

        cv::Rect2d update_box = getRect(post_state.at<float>(0, 0), post_state.at<float>(1, 0), post_state.at<float>(2, 0), post_state.at<float>(3, 0));
        last_rect = update_box;
        return last_rect;
    }

}