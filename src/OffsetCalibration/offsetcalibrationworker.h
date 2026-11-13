#pragma once
#include <QObject>
#include "interfaces/IImageService.h"
#include "interfaces/IServerProxyService.h"
#include "imageprocessor.h"
#include "headers/calibrationStorage.h"
#include "OffsetCalibration/OffsetCalibrationParameters.h"
#include "headers/calibrationStorage.h"
#include "models/UIMessageModel.h"
#include "models/ShapeModel.h"
#include <array>


using namespace imaging;

class OffsetCalibrationWorker : public QObject
{
    Q_OBJECT

public:
    OffsetCalibrationWorker(
        std::shared_ptr<IServerProxyService> proxyService,
        bool mockedImageService,
        UIMessageModel* uiMessage,
        ShapeModel* leftOverlay,
        ShapeModel* rightOverlay,
        ShapeModel* wideOverlay,
        const OffsetCalibrationParameters defaultParams,
        QObject* parent = nullptr);

    std::string m_savedImageFolder = "c:/calibration/apps/offsetCalibrationAnalysis";

    QVector<QPointF> m_polylineLeftRoi;
    QVector<QPointF> m_polylineRightRoi;
    QVector<QPointF> m_polylineWide;

    enum class ResultStatus {
        NotRun,
        NFOVPass,
        NFOVFail,
        WFOVFail,
        Pass,
        Fail
    };
    Q_ENUM(ResultStatus)

    enum class State
    {
        Idle,
        LeftRightCBOffsetCalibration,
        UVToCBOffsetCalibration,
        CalibrationVerificationCB,
        CalibrationVerification,
        WFOVOffsetCalibration,
        WFOVOffsetVerification,
        Error
    };
    Q_ENUM(State)

public slots:
    void run();
    void onImagesReady(QPixmap imageL, QPixmap imageR);
    void onStepperMoveCompleted();
    void onHomeMoveCompleted();
    void onWideFramePixmapReady(const QPixmap& image);

signals:
    void acquireImagePairRequested(IImageService::Caller, int);
    void acquireWideImageRequested(IImageService::Caller, int);
    void moveStepperRequested(double);
    void homeStepperRequested();

    void message(QString text, QColor color, QString level);
    void progressChanged(int current, int total);
    void finished(bool success = false);
    void failed(QString text);
    void statusChanged(QString text);
    void resultsReady(QString res);


    void clearNfovImages(const QPixmap& w);

private:
    QPointF cvPoint2fToQPoint(cv::Point2f &p);
    QVector<QPointF> cbCornersToPolyline(std::map<std::string,cv::Point2f>& map);
    void overlayCBCenters(const cv::Point& cbl,
                          const cv::Point& cbr,
                          float l);
    bool findCBCornersAndCenters(const cv::Mat& imageL,
                                const cv::Mat& imageR);
    bool findUVCenters(const cv::Mat& imageL,
                       const cv::Mat& imageR);

    void transformPointCoordinates(std::map<std::string, cv::Point2f>& cornerMap,
                                 cv::Point2f& cbCenter,
                                 const cv::Point vect );
    void moveToZero();

private:

    using Handler = void (OffsetCalibrationWorker::*)(
                        const cv::Mat& imageL,
                        const cv::Mat& imageR );

    void idleNoOp(const cv::Mat& imageL,
                  const cv::Mat& imageR);
    void calibrateLeftRightCBOffset(const cv::Mat& imageL,
                                    const cv::Mat& imageR);
    void calibrateUVToCBOffset(const cv::Mat& imageL,
                               const cv::Mat& imageR);
    void verifyCalibrationCB(const cv::Mat& imageL,
                             const cv::Mat& imageR);
    void verifyCalibration(const cv::Mat& imageL,
                           const cv::Mat& imageR);
    void calibrateWFOVOffset(const cv::Mat& image,
                             const cv::Mat& imageDummy);
    void verifyWFOVCalibration(const cv::Mat& image,
                               const cv::Mat& imageDummy);

    bool getRoiWithCheckerBoard(const cv::Mat& image,
                            int margin,
                            cv::Size cb_size,
                            cv::Rect& cbRoi);
    QString stateToString(OffsetCalibrationWorker::State state);

    void repotFailure(const QString status);

    QString m_resultStatus;

    State m_state = State::Idle;
    static const std::array<Handler, 7> handlers;

    std::unique_ptr<ImageProcessor> m_imageProcessor;
    std::shared_ptr<IServerProxyService> m_proxyService;
    int m_current{1};
    int m_index{0};
    int m_totalSteps{8};
    calibrationStorage  m_calStore;
    bool m_mockedHW;
    bool m_movingToZero{false};

    bool m_homedStepperOnStart{false};
    UIMessageModel* m_uiMessageModel = std::nullptr_t();
    ShapeModel* m_leftOverlay;
    ShapeModel* m_rightOverlay;
    ShapeModel* m_wideOverlay;
    bool m_doProcessing{true};
    bool m_succeeded{false};

    OffsetCalibrationParameters m_defaultParams;
    cv::Point m_halfDiff = {0,0};
    cv::Point m_offsetUV = {0,0};

    cv::Point2f m_uvCenterLeft;
    cv::Point2f m_uvCenterRight;

    cv::Point m_offsetStart;
    cv::Point m_newOffLeft;
    cv::Point m_newOffRight;

    std::map<std::string, cv::Point2f> m_cornerMapR;
    cv::Point2f m_centerR;

    std::map<std::string, cv::Point2f> m_cornerMapL;
    cv::Point2f m_centerL;

    cv::Point2f m_cbCenterL;
    cv::Point2f m_cbCenterR;

    cv::Point2f m_uvCenterInCBRelativePosition;

    cv::Point m_offsetWFOV;
    cv::Point m_uvCenterWide;
    cv::Point m_wfovRoiCenter;
    cv::Rect m_wfovRoi;
    cv::Size m_wfovDfltRoiRes{1000, 752};
    cv::Size m_wfovRes{1440, 1080};
    double m_wCameraGain{18};
    int m_wCameraExposure{10000};

    bool m_cbCentersSaved{false};
    const int m_uvDuration{300000}; // fixed 5 minutes
    const cameraTrigger m_triggerSettings{60,6};
};