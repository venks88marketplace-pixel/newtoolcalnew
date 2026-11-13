#include "headers/imageprocessor.h"
#include <QQmlEngine>
#include <QThread>
#include <QPixmap>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <QDebug>
#include <vector>
#include <math.h>

using namespace cv;
using namespace calibration;

double calculateDistance(const Point& p1, const Point& p2) {
    return cv::norm(p1-p2);
}

ImageProcessor::ImageProcessor(QObject *parent)
    : QObject{parent}
{}

bool ImageProcessor::findCBBoundingCornersAndCenter(const cv::Mat& camera,
                    const cv::Size patternSize,
                    std::map<std::string,cv::Point2f>& boundingCornerMap,
                    cv::Point2f& center) {
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCornersSB(camera, patternSize, corners, 0);
    if (found) {
        std::cout << "checkers found" << std::endl;

        center.x = center.y = 0.0; float cnt = 0.0;
        for (auto& c : corners) {
            center.x += c.x;
            center.y += c.y;
            cnt += 1.0;
        }
        if (cnt > 0) {
            center.x /= cnt;
            center.y /= cnt;
        }
        cv::Point2f c00, c01, c10, c11;
        float m00 = 10000, m01 = 0, m10 = 0, m11 = 0;
        for (auto& c : corners) {
            float m1 = c.x - c.y;
            float m2 = c.x + c.y;
            if (m2 < m00) { m00 = m2; c00 = c; }
            if (m2 > m11) { m11 = m2; c11 = c; }
            if (m1 > m01) { m01 = m1; c01 = c; }
            if (m1 < m10) { m10 = m1; c10 = c; }
        }

        boundingCornerMap.insert({ "topLeft", c00 });
        boundingCornerMap.insert({ "topRight", c01 });
        boundingCornerMap.insert({ "bottomRight", c11 });
        boundingCornerMap.insert({ "bottomLeft", c10 });

    }
    return found;
}

void ImageProcessor::drawCBRoiAndCenter( cv::Mat image,
                    EnumCameraSide cameraName,
                    const cv::Size pattern,
                    std::map<std::string,cv::Point2f>& cornerMap,
                    cv::Point2f& center){
    if (image.empty() || cornerMap.empty()) {
        qDebug() << "Empty input(s) to drawing functions";
        return;
    }
    cv::line(image,
             cornerMap["topLeft"],
             cornerMap["topRight"],
             Scalar(255, 0, 0),
             2,
             cv::LINE_AA);
    cv::line(image,
             cornerMap["topRight"],
             cornerMap["bottomRight"],
             Scalar(255, 0, 0),
             2,
             cv::LINE_AA);
    cv::line(image,
             cornerMap["topLeft"],
             cornerMap["bottomLeft"],
             Scalar(255, 0, 0),
             2,
             cv::LINE_AA);
    cv::line(image,
             cornerMap["bottomLeft"],
             cornerMap["bottomRight"],
             Scalar(255, 0, 0),
             2,
             cv::LINE_AA);

    int crossShoulder = 20;
    cv::line(image,
             Point2f{ center.x - crossShoulder, center.y },
             Point2f{ center.x + crossShoulder, center.y },
             Scalar(255, 0, 0),
             3,
             cv::LINE_AA);
    cv::line(image,
             Point2f{ center.x, center.y - crossShoulder },
             Point2f{ center.x, center.y +  crossShoulder },
             Scalar(255, 0, 0),
             3,
             cv::LINE_AA);
}

bool ImageProcessor::findUVCenter(const cv::Rect& searchRoi,
                                        const cv::Mat& image,
                                        cv::KeyPoint& uvCircle){
    cv::Mat roiImage = image(searchRoi);
    cv::imwrite("c:\\AA_temp\\uv-search-roiimage.bmp", roiImage);

    SimpleBlobDetector::Params params;
    //to accommodate UV on black checker
    params.thresholdStep = 10;
    params.minThreshold = 120;
    params.maxThreshold = 255;
    params.filterByArea = true;
    params.minArea = 20;
    params.maxArea = 2750;
    params.filterByCircularity = true;
    params.minCircularity = 0.6;

    params.filterByConvexity = false;
    params.filterByInertia = false;
    params.filterByColor = false;

    std::vector<KeyPoint> keypoints;
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    detector->detect(roiImage, CV_OUT keypoints);

    if (keypoints.size() == 0 || keypoints.size() > 1){
        qDebug() << "Failed to find single UV spot in image";
        qDebug() << "UV spots found: " << keypoints.size();
        return false;
    }
    qDebug() << "UV spot found";

    uvCircle = keypoints[0];

    return (keypoints[0].pt.x > 0 && keypoints[0].pt.y > 0);
}


