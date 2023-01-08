#include "../include/Tracking.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
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
    cv::namedWindow("tracking demo", cv::WINDOW_NORMAL);

    while (1)
    {
        cap >> frame;
        cv::imshow("tracking_demo", frame);
        cv::waitKey(1);
        if (cv::waitKey(1) == 27)
        {
            break;
        }
        /* code */
    }
    return 0;
}