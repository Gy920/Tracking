#include "../include/Tracking.hpp"

namespace Track
{

    SortTracking::SortTracking(fpGetIou fp_get_iou) : Tracking()
    {
        this->fp_get_iou_ = fp_get_iou;
        trackers_.clear();
        predict_boxs_.clear();
        tracking_result_.clear();
        is_traked_ = false;
        error_message_ = "Init";
    }

    SortTracking::~SortTracking()
    {
        this->fp_get_iou_ = NULL;
        trackers_.clear();
        predict_boxs_.clear();
        tracking_result_.clear();
        is_traked_ = false;
    }

    void SortTracking::clear()
    {
        this->fp_get_iou_ = NULL;
        trackers_.clear();
        predict_boxs_.clear();
        tracking_result_.clear();
        is_traked_ = false;
        error_message_ = "Clear";
    }

    void SortTracking::getTrakingResult(std::vector<TrackingBox> &tracking_result)
    {
        if (!is_traked_)
        {
            error_message_ = "Tracking Failed ";
        }
        tracking_result = this->tracking_result_;
    }

    std::string SortTracking::getErrorMessage()
    {
        return this->error_message_;
    }

    bool SortTracking::isTraked()
    {
        return is_traked_;
    }

    bool SortTracking::match(cv::Mat &frame, std::vector<cv::Rect2d> &detect_boxs)
    {
        if (trackers_.empty())
        {
            return false;
        }
        // get iou matrix
        unsigned int tracker_num = this->trackers_.size();
        unsigned int detect_num = detect_boxs.size();

        std::vector<std::vector<double>> iouMatrix;
        iouMatrix.resize(tracker_num, std::vector<double>(detect_num, 0));

        for (int i = 0; i < tracker_num; i++) // compute iou matrix as a distance matrix
        {
            for (int j = 0; j < detect_num; j++)
            {
                iouMatrix[i][j] = 1 - this->fp_get_iou_(this->predict_boxs_[i], detect_boxs[j]);
            }
        }

        // solve and match
        std::vector<int> assignment;
        HungarianAlgorithm HungAlgo;
        HungAlgo.Solve(iouMatrix, assignment);

        std::set<int> unmatchedDetections;
        std::set<int> unmatchedTrajectories;
        std::set<int> allItems;
        std::set<int> matchedItems;
        std::vector<cv::Point> matchedPairs;
        if (detect_num > tracker_num) //	there are unmatched detections
        {
            for (int n = 0; n < detect_num; n++)
                allItems.insert(n);

            for (int i = 0; i < tracker_num; ++i)
                matchedItems.insert(assignment[i]);

            set_difference(allItems.begin(), allItems.end(),
                           matchedItems.begin(), matchedItems.end(),
                           std::insert_iterator<std::set<int>>(unmatchedDetections, unmatchedDetections.begin()));
        }
        else if (detect_num < tracker_num) // there are unmatched predictions
        {
            for (int i = 0; i < tracker_num; ++i)
                if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
                    unmatchedTrajectories.insert(i);
        }

        for (int i = 0; i < tracker_num; ++i)
        {
            if (assignment[i] == -1) // pass over invalid values
                continue;
            if ((1 - iouMatrix[i][assignment[i]]) < IOU_THRESHOLD)
            {
                unmatchedTrajectories.insert(i);
                unmatchedDetections.insert(assignment[i]);
                // std::cout << "traker iou faied  : " << 1 - iouMatrix[i][assignment[i]] << std::endl;
            }
            else
                matchedPairs.push_back(cv::Point(i, assignment[i]));
        }

        int detIdx, trkIdx;
        for (int i = 0; i < matchedPairs.size(); i++)
        {
            trkIdx = matchedPairs[i].x;
            detIdx = matchedPairs[i].y;
            this->trackers_[trkIdx].correct(detect_boxs[detIdx]);
        }
        for (auto umd : unmatchedDetections)
        {
            BatchTracker tracker = BatchTracker(detect_boxs[umd]);
            this->trackers_.push_back(tracker);
            // std::cout << "unmatch add trakcer " << std::endl;
        }
        return true;
    }

    bool SortTracking::track(cv::Mat &frame, std::vector<cv::Rect2d> &detect_boxs)
    {
        this->tracking_result_.clear();
        this->predict_boxs_.clear();
        // init tracker
        if (trackers_.empty())
        {
            int idex = 0;
            for (auto box : detect_boxs)
            {

                BatchTracker tracker = BatchTracker(box);
                trackers_.push_back(tracker);
                TrackingBox res;
                res.box = box;
                res.id = idex++;
                tracking_result_.push_back(res);
            }

            return true;
        }

        // update trakcer , get predict box
        for (auto it = trackers_.begin(); it != trackers_.end();)
        {
            // 默认dt为1
            cv::Rect2d pBox = (*it).predict();
            if (pBox.x >= 0 && pBox.y >= 0 && pBox.height >= 0 && pBox.width >= 0)
            {
                predict_boxs_.push_back(pBox);
                it++;
            }
            else
            {
                it = trackers_.erase(it);
            }
        }

        cv::Mat frames = frame.clone();
        std::cout << " tracker " << trackers_.size() << std::endl;
        ;
        int idex = 0;
        for (auto a : trackers_)
        {
            auto box = a.getLastRect();
            /*Debug
                // std::cout<<"[Debug]: Trakcers "<<idex<<" "<<b.x<<" "<<b.y<<" "<<b.width<<" "<<b.height<<" "<<std::endl;
                // std::cout<<"[Debug]: Trackers predict "<<" idex : "<<idex<<" "<<box<<std::endl;
                // idex++;
            */
        }
        // get match
        if (!match(frame, detect_boxs))
        {
            this->error_message_ = "Match failed";
            return false;
        }
        // get output

        for (auto it = trackers_.begin(); it != trackers_.end();)
        {
            // remove dead tracklet
            if (it != trackers_.end() && (*it).since_update_age > 3)
            {
                it = trackers_.erase(it);
            }
            if (((*it).since_update_age < MAX_AGE) &&
                ((*it).m_hit_streak >= MIN_HITS))
            {
                TrackingBox res;
                res.box = (*it).getLastRect();
                res.id = (*it).id + 1;
                tracking_result_.push_back(res);
                it++;
            }
            else
            {
                /*Debug
                // std::cout<<"[Debug]: error : update age "<<(*it).since_update_age<<
                //             "  hit_streak "<<(*it).m_hit_streak<<std::endl;
                */
                it++;
            }
        }

        return true;
    }
}