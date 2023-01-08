#ifndef TRACKING_HPP_
#define TRACKING_HPP_

#include "./Common.hpp"
#include "./BatchTracker.hpp"

namespace Track
{

    /*
        The function pt to get IOU
        choice( int include/commom.hpp ):
            getIOU
            getDIOU
            getCIOU (todo)
            getGIOu (todo)
    */
    typedef double (*fpGetIou)(cv::Rect2d &box1, cv::Rect2d &box2);

    /*
        The interface base of Tracking
    */
    struct TrackingBox
    {
        cv::Rect2d box; // bounding box
        int id;         // the tracking num id ;
    };

    /*
        Class Tracking: The interface of Tracking
    */

    class Tracking
    {
    public:
        Tracking() {}

        virtual ~Tracking() {}
        // track
        virtual bool track(cv::Mat &frame, std::vector<cv::Rect2d> &detect_boxs) = 0;
        // get result
        virtual void getTrakingResult(std::vector<TrackingBox> &traking_result) = 0;
        // get error message
        virtual std::string getErrorMessage() = 0;
        // return if tracking successfull
        virtual bool isTraked() = 0;

        virtual void clear()=0;
    };

    /*
       Class SortTracking:  Sort Tracking
    */
    class SortTracking : public Tracking
    {

    public:
        SortTracking(fpGetIou fp_get_iou = getIOU);

        ~SortTracking();

        bool track(cv::Mat &frame, std::vector<cv::Rect2d> &detect_boxs) override;

        void getTrakingResult(std::vector<TrackingBox> &tracking_result) override;

        std::string getErrorMessage() override;

        bool isTraked() override;

        void clear() override;

    private:
        bool match(cv::Mat &frame, std::vector<cv::Rect2d> &detect_boxs);

    private:
        fpGetIou fp_get_iou_;

        std::vector<BatchTracker> trackers_;

        std::vector<cv::Rect2d> predict_boxs_;

        std::string error_message_;

        std::vector<TrackingBox> tracking_result_;

        bool is_traked_;

    };

    /*
        TODO
    */
    class DeepSortTracking : public Tracking
    {
        DeepSortTracking() : Tracking() {}

        virtual ~DeepSortTracking() {}
        // track
        virtual bool track(cv::Mat &frame, std::vector<cv::Rect2d> &detect_boxs) = 0;
        // get result
        virtual void getTrakingResult(std::vector<TrackingBox> &traking_result) = 0;
        // get error message
        virtual std::string getErrorMessage() = 0;
        // return if tracking successfull
        virtual bool isTraked() = 0;

        virtual void clear()=0;
    };
}
#endif