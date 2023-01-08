#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

/*Const Setting*/
#define SORT_DBL_EPSILON 0.4
#define IOU_THRESHOLD 0.3
#define MAX_AGE 1
#define MIN_HITS 3
/**/

namespace Track
{
    // 匈牙利算法
    class HungarianAlgorithm
    {
        // This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
        // The original implementation is a few mex-functions for use in MATLAB, found here:
        // http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
        //
        // Both this code and the orignal code are published under the BSD license.
        // by Cong Ma, 2016
    public:
        HungarianAlgorithm() {}
        ~HungarianAlgorithm() {}
        double Solve(std::vector<std::vector<double>> &DistMatrix, std::vector<int> &Assignment);

    private:
        void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
        void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
        void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
        void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
        void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
        void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
        void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
        void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    };

    inline double getIOU(cv::Rect2d &box1, cv::Rect2d &box2)
    {
        float inter_area = (box1 & box2).area();
        float union_area = box1.area() + box2.area() - inter_area;

        if (union_area < SORT_DBL_EPSILON)
            return 0;
        return (double)(inter_area / union_area);
    }

    inline double getDIOU(cv::Rect2d &box1, cv::Rect2d &box2)
    {
        float inter_area = (box1 & box2).area();
        cv::Rect2d union_rect = box1 | box2;
        float union_area = union_rect.area();
        if (union_area < SORT_DBL_EPSILON)
            return 0;
        cv::Point2d center1 = cv::Point2d(box1.x + box1.width / 2.0, box1.y + box1.height / 2.0);
        cv::Point2d center2 = cv::Point2d(box2.x + box2.width / 2.0, box2.y + box2.height / 2.0);
        double dis = cv::norm(center1 - center2);
        double c = cv::norm(union_rect.tl() - union_rect.br());
        return (double)(inter_area / union_area) - (double)(dis * dis / c * c);
    }

    // todo
    inline double getCIOU(cv::Rect2d &box1, cv::Rect2d &box2)
    {
        return getIOU(box1, box2);
    }

    // todo
    inline double getGIOU(cv::Rect2d &box1, cv::Rect2d &box2)
    {
        return getIOU(box1, box2);
    }
}
#endif