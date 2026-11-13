#pragma once

#include <QObject>
#include <QPair>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "utils.h"

using namespace calibration;

class ImageProcessor : public QObject
{
    Q_OBJECT
public:
    explicit ImageProcessor(QObject *parent = nullptr);

    bool findCBBoundingCornersAndCenter(const cv::Mat& camera,
            const cv::Size patternSize,
            std::map<std::string,cv::Point2f>& boundingCornerMap,
            cv::Point2f& center);

    void drawCBRoiAndCenter( cv::Mat image,
                EnumCameraSide cameraName,
                const cv::Size pattern,
                std::map<std::string,cv::Point2f>& cornermap,
                cv::Point2f& center);

    bool findUVCenter(const cv::Rect& searchRoi,
                           const cv::Mat& image,
                           cv::KeyPoint& uvSpot);

    cv::Vec3f circleDescriptor() const { return m_circleDescriptor; }

private:
    cv::Vec3f m_circleDescriptor;

};

