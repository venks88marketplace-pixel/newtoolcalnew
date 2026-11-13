#pragma once
#include <QMetaType>
#include <QDebug>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>

struct OffsetCalibrationParameters {
    double cameraExposure;
    double leftCameraGain;
    double rightCameraGain;
    unsigned int topNirLevel;
    unsigned int bottomNirLevel;
    unsigned int UVLevel;
    double zPosition_mm;
    std::pair<int, int> cameraResolution;
    std::pair<int, int> cameraRoiResolution;
    std::pair<int,int> checkerBoardSize;

    static OffsetCalibrationParameters fromJson(const QJsonObject& obj){
        if (obj.isEmpty())
            throw std::runtime_error(
                "OffsetCalibrationParams is missing or empty");
        OffsetCalibrationParameters p;
        p.cameraExposure =
            obj.value("exposure").toDouble(p.cameraExposure);
        p.leftCameraGain =
            obj.value("gain_left_camera").toDouble(p.leftCameraGain);
        p.rightCameraGain =
            obj.value("gain_right_camera").toDouble(p.rightCameraGain);
        p.topNirLevel =
            obj.value("nir_top").toInt(p.topNirLevel);
        p.bottomNirLevel =
            obj.value("nir_bottom").toInt(p.bottomNirLevel);
        p.UVLevel =
            obj.value("uv_level").toInt(p.UVLevel);
        p.zPosition_mm =
            obj.value("z_position_mm").toDouble(p.zPosition_mm);
        auto camResArray = obj.value("camera_resolution").toArray();
        p.cameraResolution = std::pair<int,int>(camResArray[0].toInt(),
                                                 camResArray[1].toInt());
        auto camRoiResArray = obj.value("camera_roi_resolution").toArray();
        p.cameraRoiResolution = std::pair<int,int>(camRoiResArray[0].toInt(),
                                                    camRoiResArray[1].toInt());
        auto cbArray = obj.value("cb_pattern_size").toArray();
        p.checkerBoardSize = std::pair<int,int>(cbArray[0].toInt(),
                                                    cbArray[1].toInt());
        return p;
    }
};
Q_DECLARE_METATYPE(OffsetCalibrationParameters);