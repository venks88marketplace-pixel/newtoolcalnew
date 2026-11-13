#pragma once
#include <QObject>
#include <filesystem>
#include <opencv2/core.hpp>

class IACalibrator : public QObject
{
    Q_OBJECT
public:
    explicit IACalibrator(cv::Size patternSize, std::filesystem::path analysisFolder, QObject *parent = nullptr);
    bool getCheckerboardCorners(const cv::Mat& imgLeft, const cv::Mat& imgRight);
    bool trainIA(const cv::Mat& imgLeft, const cv::Mat& imgRight);
    QImage createImage(int radius);

signals:

private:


    cv::Size m_patternSize;
    std::filesystem::path m_analysisFolder;

    std::vector<cv::Point3f> m_cbFrame;
    std::vector<cv::Point3f> m_cbFrameFlat;
    cv::Point3f m_center;
    cv::Mat m_toIA;

    std::array<cv::Point2f, 4> m_checkerCornersLeft;
    std::array<cv::Point2f, 4> m_checkerCornersRight;

    void na2ia();
};

