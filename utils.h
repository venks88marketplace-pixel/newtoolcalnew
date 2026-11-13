#pragma once
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <string_view>
#include <QJsonObject>
#include <QJsonArray>
#include <QString>
#include <QImage>
#include <QPixmap>
#include <QMetaEnum>
#include <opencv2/core.hpp>

namespace calibration {

    enum class EnumCameraSide{ LeftCam, RightCam, WFOVcam };

    struct
        Parameter {
        bool trained = false;
        bool useRectification = true;
        cv::Mat K1, K2;
        cv::Mat D1, D2;
        cv::Mat Pex1, Pex2;
        cv::Mat rvec1, rvec2, tvec1, tvec2;
        cv::Mat R, T, E, F;
        cv::Mat R1, R2, P1, P2, Q;
    };

    inline constexpr std::string_view CamToString(EnumCameraSide camside) {
        switch (camside) {
        case EnumCameraSide::LeftCam:
            return "LeftCamera";
        case EnumCameraSide::RightCam:
            return "RightCamera";
        case EnumCameraSide::WFOVcam:
            return "WFOVCamera";
        }
        return {};
    }

    inline cv::Mat QPixmapToGrayMat(const QPixmap& pixmap){
        QImage image = pixmap.toImage().convertToFormat(QImage::Format_Grayscale8);
        return cv::Mat(
                   image.height(),
                   image.width(),
                   CV_8UC1,
                   const_cast<uchar*>(image.bits()),
                   image.bytesPerLine()).clone();
    }

    inline cv::Mat QImageToCvMat(const QImage &image)
    {
        QImage converted = image.convertToFormat(QImage::Format_Grayscale8);
        return cv::Mat(converted.height(), converted.width(), CV_8UC1,
            const_cast<uchar*>(converted.bits()), converted.bytesPerLine()).clone();
    }

    inline std::string CurrentDateString() {
        auto p = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(p);
        std::tm tm{};
        localtime_s(&tm, &t);
        std::stringstream ss;
        ss << std::put_time(&tm,"%Y_%b_%d_%H_%M_%S");
        return ss.str();
    }

    inline cv::Mat stretch(cv::Mat in){
        double minV = 0.0, maxV = 0.0;
        cv::minMaxLoc(in, &minV, &maxV);

        cv::Mat stretched;
        if (maxV > minV) {
            in.convertTo(stretched, CV_8U, 255.0 / (maxV - minV), -minV * 255.0 / (maxV - minV));
        } else {
            stretched = in.clone();
        }
        return stretched;
    }

    inline QPixmap matToPixmap(const cv::Mat& m) {
        if (m.empty()) return QPixmap();
        if (m.type() == CV_8UC3) {
            cv::Mat rgb;
            cv::cvtColor(m, rgb, cv::COLOR_BGR2RGB);
            QImage q(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
            return QPixmap::fromImage(q.copy());
        }
        if (m.type() == CV_8UC1) {
            QImage q(m.data, m.cols, m.rows, static_cast<int>(m.step), QImage::Format_Grayscale8);
            return QPixmap::fromImage(q.copy());
        }
        return QPixmap();
    }


    inline bool ends_with(const std::string& main_str, const std::string& suffix) {
        if (suffix.size() > main_str.size()) return false;

        // Compare the end portion of main_str with the suffix
        return main_str.compare(main_str.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    //json helpers
    // QJsonObject requireObject(const QJsonObject& parent,
    //                           const QString& key){
    //     auto it = parent.find(key);

    //     if (it == parent.end() || !it->isObject())
    //         throw std::runtime_error(
    //             QString("Missing object '%1'")
    //                 .arg(key).toStdString());

    //     return it->toObject();
    // }

    // QJsonArray requireArray(const QJsonObject& parent,
    //                                 const QString& key){
    //     auto it = parent.find(key);

    //     if (it == parent.end() || !it->isArray())
    //         throw std::runtime_error(
    //             QString("Missing array '%1'")
    //                 .arg(key).toStdString());

    //     return it->toArray();
    // }
    }
