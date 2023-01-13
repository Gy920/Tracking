#include "../include/Tracking.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <chrono>
#include <iostream>

void getDetectBox(int idex, std::vector<cv::Rect2d> &detect_box)
{
    detect_box.clear();
    if (idex >= 15 && idex <= 53 && idex % 2 == 1) //每两帧检测到一次
    {
        detect_box.push_back(cv::Rect2d(0 + 25 * (idex - 15), 250, 320, 230));
    }
    if (idex >= 42 && idex <= 72 && idex % 3 != 1) //每三帧检测到一次
    {
        detect_box.push_back(cv::Rect2d(0 + 32 * (idex - 42), 80, 290, 200));
    }
}

// 红色为detect box ,蓝色为 correct box
int main(int argc, char *argv[])
{

    if (argc != 2)
    {
        std::cout << "[ERROT], input error. (like : \" ./demo  ../test_video.mp4 \") " << std::endl;
        exit(1);
    }
    std::shared_ptr<Track::Tracking> trakcer;
    trakcer = std::make_shared<Track::SortTracking>(Track::getIOU);

    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened())
    {
        std::cout << "[ERROT], VIDEO STREAM IS EMPTY " << std::endl;
        exit(1);
    }
    cv::Mat frame;
    cap >> frame;
    cv::namedWindow("tracking_demo", cv::WINDOW_NORMAL);
    std::cout << frame.cols << " " << frame.rows << std::endl;
    int idex = 0;
    std::vector<cv::Rect2d> detect_box;
    std::vector<Track::TrackingBox> update_box;

    while (1)
    {
        idex++;
        cap >> frame;
        if (frame.empty() || idex == 73)
        {
            break;
        }
        std::cout << idex << std::endl;
        //
        getDetectBox(idex, detect_box);
        //
        trakcer->track(frame, detect_box);
        trakcer->getTrakingResult(update_box);

        for (auto a : detect_box)
        {
            cv::rectangle(frame, a, cv::Scalar(255, 0, 255), 5);
        }
        for (auto a : update_box)
        {
            cv::rectangle(frame, a.box, cv::Scalar(255, 0, 0), 2);
            cv::putText(frame,std::to_string(a.id),a.box.tl(),2,2,cv::Scalar(255, 0, 0), 2);
        }
        cv::imshow("tracking_demo", frame);
        if (cv::waitKey(100) == 27)
        {
            break;
        }
        /* code */
    }
    return 0;
}
