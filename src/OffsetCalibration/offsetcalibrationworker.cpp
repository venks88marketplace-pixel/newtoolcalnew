#include "OffsetCalibration/offsetcalibrationworker.h"
#include <QTimer>
#include <QtCore>
#include <QVector>
#include <QPoint>
#include <QPointF>
#include "utils.h"
#include <opencv2/calib3d.hpp>

using namespace cv;

OffsetCalibrationWorker::OffsetCalibrationWorker(
    std::shared_ptr<IServerProxyService> proxyService,
    bool mockedHW,
    UIMessageModel* uiMessageModel,
    ShapeModel* leftOverlay,
    ShapeModel* rightOverlay,
    ShapeModel* wideOverlay,
    const OffsetCalibrationParameters defaultParams,
    QObject* parent)
    : QObject(parent),
    m_defaultParams(defaultParams),
    m_proxyService(std::move(proxyService)),

    m_mockedHW(mockedHW),
    m_uiMessageModel(uiMessageModel),
    m_leftOverlay(leftOverlay),
    m_rightOverlay(rightOverlay),
    m_wideOverlay(wideOverlay){
    m_imageProcessor = std::make_unique<ImageProcessor>(parent);
}

const std::array<OffsetCalibrationWorker::Handler, 7>
    OffsetCalibrationWorker::handlers = {{
        &OffsetCalibrationWorker::idleNoOp,
        &OffsetCalibrationWorker::calibrateLeftRightCBOffset,
        &OffsetCalibrationWorker::calibrateUVToCBOffset,
        &OffsetCalibrationWorker::verifyCalibrationCB,
        &OffsetCalibrationWorker::verifyCalibration,
        &OffsetCalibrationWorker::calibrateWFOVOffset,
        &OffsetCalibrationWorker::verifyWFOVCalibration
    }};

void OffsetCalibrationWorker::run(){
    calibrationStorage  calstore;
    std::string dataRootOffset = calstore.calDataRoot().string()
                                      + "OffsetCalibrationAnalysis//";
    qDebug() << "Offset calibration: starting run 1";

    if (!m_mockedHW) {
        m_proxyService->setCameraGainsAndExposure(
            m_defaultParams.leftCameraGain,
            m_defaultParams.rightCameraGain,
            m_defaultParams.cameraExposure);

        m_proxyService->setWideCameraGainAndExposure(
            m_wCameraGain,
            m_wCameraExposure);

        m_proxyService->setCameraROIs(
            m_defaultParams.cameraResolution.first,
            m_defaultParams.cameraResolution.second,
            0,
            0,
            0,
            0);

        m_proxyService->setWFOVCameraROI(
            m_wfovRes.width,
            m_wfovRes.height,
            0,
            0);

        m_proxyService->setNIR(
            m_defaultParams.topNirLevel,
            m_defaultParams.bottomNirLevel);

        m_proxyService->setCameraTrigger(true,
            m_triggerSettings.frequency_hz,
            m_triggerSettings.duration_ms);

        m_proxyService->setUVOff();

        emit statusChanged(QString("Homing XYZ motors..."));
        m_proxyService->homeStageMotors(); //xyz motors
        //need to pause here for the motors to settle
        QThread::sleep(8);
    }

    emit progressChanged(m_current, m_totalSteps);
    m_state = State::Idle;

    emit statusChanged(QString("Moving stage to 11 mm"));
    emit moveStepperRequested(m_defaultParams.zPosition_mm);
}

void OffsetCalibrationWorker::moveToZero() {
    m_movingToZero = true;
    emit statusChanged(QString("Moving motor to position 0 mm"));
    emit moveStepperRequested(0);
}

void OffsetCalibrationWorker::onStepperMoveCompleted() {
    qDebug() << "stepper move completed";
    m_current++;
    if (m_movingToZero) {
        emit progressChanged(m_totalSteps, m_totalSteps);
        emit statusChanged(QString("Calibration completed."));
        emit finished(m_state != State::Error);
    } else {
        emit progressChanged(m_current, m_totalSteps);
        m_state = State::LeftRightCBOffsetCalibration;
        qDebug()  << "emmiting  acquireImagePairRequested";
        emit acquireImagePairRequested(IImageService::Caller::OffsetCal, m_index);
    }
}

void OffsetCalibrationWorker::onHomeMoveCompleted() {
    qDebug() << "Homing stepper move completed";
    m_current++;
    emit progressChanged(m_current, m_totalSteps);
    emit statusChanged(QString("Home completed"));
    emit moveStepperRequested(m_defaultParams.zPosition_mm);
}

void OffsetCalibrationWorker::onImagesReady(QPixmap imageL,
                                         QPixmap imageR) {
    qDebug() << "NFOV image ready received";
    cv::Mat limage = calibration::QPixmapToGrayMat(imageL);
    cv::Mat rimage = calibration::QPixmapToGrayMat(imageR);
    if (limage.rows == 0 || rimage.rows == 0) {
        qDebug() << "invalid images";
        m_resultStatus = "Fail";
        repotFailure(m_resultStatus);
        return;
    }
    else{
        const auto index = static_cast<size_t>(m_state);
        (this->*handlers.at(index))(limage, rimage);
    }
}

void OffsetCalibrationWorker::onWideFramePixmapReady(const QPixmap& img)
{
    qDebug() << "WFOV Frame Pixmap image ready received";
    cv::Mat image = calibration::QPixmapToGrayMat(img);

    if (image.rows == 0 || image.rows == 0) {
        qDebug() << "Invalid WFOV Frame Pixmap image";
        m_resultStatus = "WFOV Fail";
        repotFailure(m_resultStatus);
        return;
    }
    else{
        const auto index = static_cast<size_t>(m_state);
        (this->*handlers.at(index))(image, cv::Mat());
    }
}

void OffsetCalibrationWorker::calibrateLeftRightCBOffset(const cv::Mat& imageL,
                                                         const cv::Mat& imageR){
    qDebug() << "NFOV calibrate Left-Right";
    emit statusChanged(
        QString("Computing offset of left and right CB centers"));
    if (!findCBCornersAndCenters(imageL, imageR)){
        qDebug() << "Failure finding checkerboard features.";
        m_resultStatus = "NFOV Fail";
        repotFailure(m_resultStatus);
        return;
    }
    cv::Point2f imageCenter = {m_defaultParams.cameraResolution.first/2.0f,
                             m_defaultParams.cameraResolution.second /2.0f};

    //found CB points need to be centered in 2016x1452 ROIs
    //left image
    int x_offset = {(static_cast<int>(std::round(m_cbCenterL.x)) -
                    m_defaultParams.cameraRoiResolution.first / 2)};
    int y_offset = {(static_cast<int>(std::round(m_cbCenterL.y)) -
                    m_defaultParams.cameraRoiResolution.second / 2)};
    m_newOffLeft = {x_offset, y_offset};

    //right image
    x_offset = {(static_cast<int>(std::round(m_cbCenterR.x)) -
                     m_defaultParams.cameraRoiResolution.first / 2)};
    y_offset = {(static_cast<int>(std::round(m_cbCenterR.y)) -
                     m_defaultParams.cameraRoiResolution.second / 2)};
    m_newOffRight = {x_offset, y_offset};

    qDebug() << "saving first offsets to the calibration file";
    m_calStore.updateNfovOffsets(m_newOffLeft.x,
                                 m_newOffLeft.y,
                                 m_newOffRight.x,
                                 m_newOffRight.y);

    //apply offsets to CB corners
    transformPointCoordinates(m_cornerMapL, m_cbCenterL, m_newOffLeft);
    transformPointCoordinates(m_cornerMapR, m_cbCenterR, m_newOffRight);

    //prepare for acquiring images with the checkerboards centered in ROI
    m_proxyService->setCameraROIs(
        m_defaultParams.cameraRoiResolution.first,
        m_defaultParams.cameraRoiResolution.second,
        m_newOffLeft.x,
        m_newOffLeft.y,
        m_newOffRight.x,
        m_newOffRight.y);

    qDebug() << "ROI: "
             << "W-H: (" <<  m_defaultParams.cameraRoiResolution.first
             << ", " << m_defaultParams.cameraRoiResolution.second << ")";
    qDebug() << "left ROI center: "
             << "center: (" << m_cbCenterL.x << ", " << m_cbCenterL.y << ")";
    qDebug() << "left ROI offset: "
             << "offset: (" << m_newOffLeft.x << ", " << m_newOffLeft.y << ")";
    qDebug() << "NEXT state is set to UVToCBOffsetCalibration";

    m_state = State::UVToCBOffsetCalibration;

    m_proxyService->setUV(m_defaultParams.UVLevel, m_uvDuration);
    m_proxyService->setNIR(
        m_defaultParams.topNirLevel/2,
        m_defaultParams.bottomNirLevel/2);
    m_current++;
    emit progressChanged(m_current, m_totalSteps);
    emit statusChanged(QString("calibrateLeftRightCBOffset completed"));

    m_proxyService->setDmdImage(QImage(":/images/dotImage_4_1x1.bmp"));

    //allow time to view results
    QThread::sleep(3);
    m_leftOverlay->clear();
    m_rightOverlay->clear();
    emit acquireImagePairRequested(IImageService::Caller::OffsetCal, m_index);
}

void OffsetCalibrationWorker::calibrateUVToCBOffset(const cv::Mat& imageL, const cv::Mat& imageR){
    qDebug() << "calibrating UV-2-CB";

    m_polylineLeftRoi = cbCornersToPolyline( m_cornerMapL);
    m_polylineRightRoi = cbCornersToPolyline( m_cornerMapR);

    m_leftOverlay-> addPolyline(m_polylineLeftRoi, QColor("blue"));
    m_rightOverlay-> addPolyline(m_polylineRightRoi, QColor("blue"));
    overlayCBCenters(m_cbCenterL, m_cbCenterR, 30.0);

    QThread::sleep(2);

    emit statusChanged(QString("calibrating UV Offset"));
    if (!findUVCenters(imageL, imageR)){
        qDebug() << "Failure finding UV spots.";
        m_resultStatus = "NFOV Fail";
        repotFailure(m_resultStatus);
        return;
    }

    //UV coordinates in left and right images are practicaly equal
    cv::Point uvToCBdiffL = m_uvCenterLeft - m_cbCenterL;
    cv::Point uvToCBdiffR = m_uvCenterRight - m_cbCenterR;
    cv::Point2f uvToCBdiff = (uvToCBdiffR + uvToCBdiffL)/2.0;

    qDebug() << "uvToCBdiff " << uvToCBdiff.x << " ," << uvToCBdiff.y;
    qDebug() << "old offL" << m_newOffLeft.x << " ," << m_newOffLeft.y;

    m_newOffLeft += static_cast<cv::Point>(cv::Point(std::round(uvToCBdiff.x), std::round(uvToCBdiff.y)));
    m_newOffRight += static_cast<cv::Point>(cv::Point(std::round(uvToCBdiff.x), std::round(uvToCBdiff.y)));

    qDebug() << "new offL" << m_newOffLeft.x << " ," << m_newOffLeft.y;

    qDebug() << "saving first offsets to the calibration file";
    m_calStore.updateNfovOffsets(m_newOffLeft.x,
                                 m_newOffLeft.y,
                                 m_newOffRight.x,
                                 m_newOffRight.y);

    m_proxyService->setCameraROIs(
        m_defaultParams.cameraRoiResolution.first,
        m_defaultParams.cameraRoiResolution.second,
        m_newOffLeft.x,
        m_newOffLeft.y,
        m_newOffRight.x,
        m_newOffRight.y);

    QThread::sleep(5);

    qDebug() << "NEXT state is CalibrationVerificationCB";
    m_state = State::CalibrationVerificationCB;

    m_proxyService->setUVOff();
    m_proxyService->setNIR(
        m_defaultParams.topNirLevel,
        m_defaultParams.bottomNirLevel);

    emit progressChanged(++m_current, m_totalSteps);
    emit statusChanged(QString("calibrateUVToCBOffset completed"));
    emit acquireImagePairRequested(IImageService::Caller::OffsetCal, m_index);
}

void OffsetCalibrationWorker::verifyCalibrationCB(const cv::Mat& imageL, const cv::Mat& imageR){
    qDebug() << "verifying CB cal";
    qDebug() << "current m_state: " << stateToString(m_state);

    emit statusChanged(QString("Verifying Left-Right calibration"));

    qDebug() << "old m_cbCenterL" << m_cbCenterL.x << " ," << m_cbCenterL.y;
    qDebug() << "old m_cbCenterR" << m_cbCenterR.x << " ," << m_cbCenterR.y;

    qDebug() << "new m_cbCenterL" << m_cbCenterL.x << " ," << m_cbCenterL.y;
    qDebug() << "new m_cbCenterR" << m_cbCenterR.x << " ," << m_cbCenterR.y;

    overlayCBCenters(m_cbCenterL, m_cbCenterR, 30.0);

    //setup UV verification
    qDebug() << "NEXT state is CalibrationVerification";
    m_state = State::CalibrationVerification;
    m_proxyService->setUV(m_defaultParams.UVLevel, m_uvDuration);
    m_proxyService->setNIR(
        m_defaultParams.topNirLevel/2,
        m_defaultParams.bottomNirLevel/2);
    emit progressChanged(++m_current, m_totalSteps);
    emit statusChanged(QString("CB verification completed"));

    m_proxyService->setDmdImage(QImage(":/images/dotImage_4_1x1.bmp"));

    //allow time to view results
    QThread::sleep(3);
    emit acquireImagePairRequested(IImageService::Caller::OffsetCal, m_index);
}

void OffsetCalibrationWorker::verifyCalibration(const cv::Mat& imageL, const cv::Mat& imageR){
    qDebug() << "verifying NFOV calibration";
    qDebug() << "current m_state: " << stateToString(m_state);

    if ( !findUVCenters(imageL, imageR)){
        qDebug() << "failed finding UV in cal verification";
        m_resultStatus = "NFOV Fail";
        repotFailure(m_resultStatus);
        return;
    }
    emit statusChanged(QString("verifying Offset calibration"));
    m_proxyService->setUVOff();
    emit progressChanged(++m_current, m_totalSteps);
    emit statusChanged(QString("NFOV offset calibration verification completed"));
    //m_state = State::Idle;
    QThread::sleep(3);

    //spread of CB four points (extreme ones) using Left camera
    int x_span = m_cornerMapL["topRight"].x - m_cornerMapL["topLeft"].x;
    int y_span = m_cornerMapL["bottomLeft"].y - m_cornerMapL["topLeft"].y;

    CV_Assert(x_span != 0.0f && y_span != 0.0f);

    m_uvCenterInCBRelativePosition =
        cv::Point2f((m_uvCenterLeft.x - m_cornerMapL["topLeft"].x ) / x_span,
        (m_uvCenterLeft.y - m_cornerMapL["topLeft"].y ) / y_span);

    m_resultStatus = "NFOV Pass";
    emit resultsReady(m_resultStatus);

    qDebug() << "NEXT state is WFOVCameraROI";

    m_state = State::WFOVOffsetCalibration;
    m_proxyService->setWFOVCameraROI(
        m_wfovRes.width,
        m_wfovRes.height,
        0,
        0);

    m_proxyService->setNIR(0,0);

    emit statusChanged(QString("calibrating WFOVOffset - REMOVE SHROUD!!!"));

    QThread::sleep(25); //need time to remove light blocker shroud

    emit acquireWideImageRequested(IImageService::Caller::OffsetCal, m_index);
}

void OffsetCalibrationWorker::calibrateWFOVOffset(const cv::Mat& image, const cv::Mat& imageDummy){
    qDebug() << "in calibrateWFOVOffset";
    qDebug() << "current m_state: " << stateToString(m_state);
    //imageDummy is used only for API reuse

    cv::Size cb_size{m_defaultParams.checkerBoardSize.first,
                     m_defaultParams.checkerBoardSize.second};

    m_wideOverlay-> addCircle(700, 500, 30,
                             qRgba(0,255,0,0));

    cv::Rect cbRoi;
    int margin = 75;
    if (!getRoiWithCheckerBoard(image, margin, cb_size, cbRoi)){
        qDebug() << "Failed detecting CB in WFOV cal";
        m_resultStatus = "WFOV Fail";
        repotFailure(m_resultStatus);
        return;
    }
    cv::Mat roiImage = image(cbRoi);
    std::map<std::string, cv::Point2f> cbCornerMap;
    cv::Point2f cbCenter;

    if( !m_imageProcessor->findCBBoundingCornersAndCenter(roiImage,
                                                          cb_size,
                                                          cbCornerMap,
                                                          cbCenter)) {
        qDebug() << "WFOV: failed to find checkerboard.";
        m_resultStatus = "WFOV Fail";
        repotFailure(m_resultStatus);
        return;
    }
    cv::Point2f roiOffset = static_cast<cv::Point2f>(cv::Point(cbRoi.x, cbRoi.y));
    cbCornerMap["topLeft"] += roiOffset;
    cbCornerMap["topRight"] += roiOffset;
    cbCornerMap["bottomLeft"] += roiOffset;
    cbCornerMap["bottomRight"] += roiOffset;
    cbCenter  += roiOffset;

    qDebug() << "WFOV CB: "
             << "topLeft" << cbCornerMap["topLeft"].x << ", " << cbCornerMap["topLeft"].y << ")" << "\n"
             << "topRight" << cbCornerMap["topRight"].x << ", " << cbCornerMap["topRight"].y << ")" << "\n"
             << "bottomLeft" << cbCornerMap["bottomLeft"].x << ", " << cbCornerMap["bottomLeft"].y << ")" << "\n"
             << "bottomRight" << cbCornerMap["bottomRight"].x << ", " << cbCornerMap["bottomRight"].y << ")";


    m_polylineWide = cbCornersToPolyline( cbCornerMap);
    m_wideOverlay->addPolyline(m_polylineWide, QColor("orange"));

    qDebug() << "WFOV: FOUND checkerboard.";

    float uvXfromNFOV = (cbCornerMap["topRight"].x - cbCornerMap["topLeft"].x) *
                            m_uvCenterInCBRelativePosition.x + cbCornerMap["topLeft"].x;

    float uvYfromNFOV = (cbCornerMap["bottomLeft"].y - cbCornerMap["topLeft"].y) *
                            m_uvCenterInCBRelativePosition.y + cbCornerMap["topLeft"].y;

    m_uvCenterWide = cv::Point(cvRound(uvXfromNFOV),cvRound(uvYfromNFOV));

    m_wfovRoiCenter = cv::Point(uvXfromNFOV, uvYfromNFOV);

    m_offsetWFOV = cv::Point(m_wfovRoiCenter.x - m_wfovDfltRoiRes.width / 2,
                             m_wfovRoiCenter.y - m_wfovDfltRoiRes.height / 2);
    m_wfovRoi = cv::Rect(m_offsetWFOV.x, m_offsetWFOV.y,
                         m_wfovDfltRoiRes.width, m_wfovDfltRoiRes.height);

    m_proxyService->setWFOVCameraROI(
        m_wfovDfltRoiRes.width,
        m_wfovDfltRoiRes.height,
        m_offsetWFOV.x,
        m_offsetWFOV.y);

    qDebug() << "NEXT state is: WFOVOffsetVerification";
    m_state = State::WFOVOffsetVerification;

    QThread::sleep(3);
    emit acquireWideImageRequested(IImageService::Caller::OffsetCal, m_index);
}

void OffsetCalibrationWorker::verifyWFOVCalibration(const cv::Mat& image, const cv::Mat& imageDummy){
    qDebug() << "In verify WFOVOffset";
    qDebug() << "current m_state: " << stateToString(m_state);

    qDebug() << "verifying WFOV calibration";
    m_wideOverlay->clear();
    m_wideOverlay-> addCircle(m_uvCenterWide.x, m_uvCenterWide.y, 10,
                             qRgba(0,255,0,0));

    m_wideOverlay->addRect(m_wfovRoi.x, m_wfovRoi.y,
                           m_wfovRoi.width, m_wfovRoi.height );
    int l = 35;
    m_wideOverlay-> addLine(m_wfovRoiCenter.x - l, m_wfovRoiCenter.y, m_wfovRoiCenter.x + l, m_wfovRoiCenter.y,
                           QColor("darkgoldenrod"));
    m_wideOverlay-> addLine(m_wfovRoiCenter.x, m_wfovRoiCenter.y - l, m_wfovRoiCenter.x, m_wfovRoiCenter.y + l,
                           QColor("darkgoldenrod"));

    //cal file
    m_calStore.updateWFOVParameters(m_offsetWFOV.x, m_offsetWFOV.y);

    m_resultStatus = "Succeeded";
    emit resultsReady(m_resultStatus);
    m_movingToZero = true;
    emit finished(true);
    moveToZero();
}

void OffsetCalibrationWorker::idleNoOp(const cv::Mat& imageL, const cv::Mat& imageR){
}

bool OffsetCalibrationWorker::findCBCornersAndCenters(const cv::Mat& imageL,
                                                      const cv::Mat& imageR){
    cv::Size cb_size{m_defaultParams.checkerBoardSize.first,
                     m_defaultParams.checkerBoardSize.second};

    m_cornerMapL.clear();
    m_cornerMapR.clear();
    m_leftOverlay->clear();
    m_rightOverlay->clear();
    bool found_l, found_r;
    try {
        //enchance image to boost contrast
        cv::Mat imageL_n;
        cv::normalize(
            imageL,
            imageL_n,
            0,
            255,
            cv::NORM_MINMAX,
            CV_8U
            );
        found_l = m_imageProcessor->findCBBoundingCornersAndCenter(imageL_n,
                                                                    cb_size,
                                                                    m_cornerMapL,
                                                                    m_cbCenterL);
        cv::Mat imageR_n;
        cv::normalize(
            imageR,
            imageR_n,
            0,
            255,
            cv::NORM_MINMAX,
            CV_8U
            );

        found_r = m_imageProcessor->findCBBoundingCornersAndCenter(imageR_n,
                                                                    cb_size,
                                                                    m_cornerMapR,
                                                                    m_cbCenterR);
        if( found_l && found_r ) {
            m_polylineLeftRoi = cbCornersToPolyline( m_cornerMapL);
            m_polylineRightRoi = cbCornersToPolyline( m_cornerMapR);

            m_leftOverlay-> addPolyline(m_polylineLeftRoi, QColor("orange"));
            m_rightOverlay-> addPolyline(m_polylineRightRoi, QColor("orange"));
        }
    }
    catch(cv::Exception& e){
        qDebug() << "Failed finding CB features.";
        return false;
    }
    return found_l && found_r;
}

//legitimate allowed locations of UV center spot are constrained
//by a maximum size of circle projected by DMD.
//Checkerboard target has known checker size in mm.
//So we can express radius of the circle by number of checkers.
//TODO: add that computation here. For now, hardcoded search area
//to exclude UV bright extreneous areas.
bool OffsetCalibrationWorker::findUVCenters(const cv::Mat& imageL,
                                            const cv::Mat& imageR){
    m_rightOverlay->clear();
    m_leftOverlay->clear();

    //search window for UV spot detection is configured to exclude extraneous UV glare
    cv::Rect searchRoi(m_cbCenterL.x - 200,
                       m_cbCenterL.y - 200,
                       400,
                       400);

    QVector<QPointF> searchPolyline{QPointF(searchRoi.x, searchRoi.y),
                                    QPointF(searchRoi.x + searchRoi.width, searchRoi.y),
                                    QPointF(searchRoi.x + searchRoi.width, searchRoi.y + searchRoi.height),
                                    QPointF(searchRoi.x, searchRoi.y + searchRoi.height),
                                    QPointF(searchRoi.x, searchRoi.y)};
    m_leftOverlay-> addPolyline(
        searchPolyline,
        QColor("green"), 4);
    m_rightOverlay-> addPolyline(
        searchPolyline,
        QColor("green"), 4);

    overlayCBCenters(m_cbCenterL, m_cbCenterR, 30.0);
    cv::KeyPoint uvLeft, uvRight;
    bool foundL = m_imageProcessor->findUVCenter(searchRoi,
                                    imageL,
                                    uvLeft);
    bool foundR = m_imageProcessor->findUVCenter(searchRoi,
                                     imageR,
                                     uvRight);
    if ( !foundL || !foundR) {
        qDebug() << "Failure finding UV spot in NFOV images";
        return false;
    }
    m_uvCenterLeft = cv::Point2f(uvLeft.pt.x + searchRoi.x,
                        uvLeft.pt.y + searchRoi.y);

    m_uvCenterRight = cv::Point2f(uvRight.pt.x + searchRoi.x,
                                 uvRight.pt.y + searchRoi.y);
    qDebug() << "UV Center Left: "
             << "cv::Point(" << m_uvCenterLeft.x << ", " << m_uvCenterLeft.y << ")";
    qDebug() << "CBoard Center Right: "
             << "cv::Point(" << m_uvCenterRight.x << ", " << m_uvCenterRight.y << ")";
    overlayCBCenters(m_cbCenterL, m_cbCenterR, 30.0);

    m_leftOverlay-> addCircle(m_uvCenterLeft.x, m_uvCenterLeft.y, uvLeft.size,
                                                            qRgba(0,255,0,0));
    m_rightOverlay-> addCircle(m_uvCenterRight.x, m_uvCenterRight.y, uvRight.size,
                                                            qRgba(0,255,0,0));
    return true;
}

QPointF OffsetCalibrationWorker::cvPoint2fToQPoint(cv::Point2f &p) {
    return QPointF(static_cast<double>(p.x), static_cast<double>(p.y));
}

QVector<QPointF> OffsetCalibrationWorker::cbCornersToPolyline(std::map<std::string,cv::Point2f>& map){
    QVector<QPointF> pl;
    pl.append(cvPoint2fToQPoint(map["topLeft"]));
    pl.append(cvPoint2fToQPoint(map["topRight"]));
    pl.append(cvPoint2fToQPoint(map["bottomRight"]));
    pl.append(cvPoint2fToQPoint(map["bottomLeft"]));
    pl.append(cvPoint2fToQPoint(map["topLeft"]));
    return pl;
}

void OffsetCalibrationWorker::overlayCBCenters(const cv::Point& cbl,
                                               const cv::Point& cbr,
                                               float l){
    m_leftOverlay-> addLine(cbl.x - l, cbl.y, cbl.x + l, cbl.y,
                               QColor("darkgoldenrod"));
    m_leftOverlay-> addLine(cbl.x, cbl.y - l, cbl.x, cbl.y + l,
                           QColor("darkgoldenrod"));
    m_rightOverlay-> addLine(cbr.x - l, cbr.y, cbr.x + l, cbr.y,
                            QColor("darkgoldenrod"));
    m_rightOverlay-> addLine(cbr.x, cbr.y - l, cbr.x, cbr.y + l,
                            QColor("darkgoldenrod"));
}

void OffsetCalibrationWorker::transformPointCoordinates(
                    std::map<std::string,cv::Point2f>& cornerMap,
                                            cv::Point2f& cbCenter,
                                        const cv::Point offset ){
    cv::Point2f val = static_cast<cv::Point2f>(offset);
    qDebug() << "cbMap topLeft before: " << cornerMap["topLeft"].x << " ," << cornerMap["topLeft"].y;
    qDebug() << "cbMap bottomRight before: " << cornerMap["bottomRight"].x << " ," << cornerMap["bottomRight"].y;
    qDebug() << "cbMap center before: " << cbCenter.x << " ," << cbCenter.y;

    cornerMap["topLeft"] -= val;
    cornerMap["topRight"] -= val;
    cornerMap["bottomLeft"] -= val;
    cornerMap["bottomRight"] -= val;
    cbCenter -= val;

    qDebug() << "cbMap topLeft after: " << cornerMap["topLeft"].x << " ," << cornerMap["topLeft"].y;
    qDebug() << "cbMap bottomRight before: " << cornerMap["bottomRight"].x << " ," << cornerMap["bottomRight"].y;
    qDebug() << "cbMap center before: " << cbCenter.x << " ," << cbCenter.y;
}

bool OffsetCalibrationWorker::getRoiWithCheckerBoard(const cv::Mat& image,
                                                int margin,
                                                cv::Size cb_size,
                                                cv::Rect& cbRoi) {
    std::vector<cv::Point2f> corners;
    if (!cv::findChessboardCornersSB(image, cb_size, corners)){
        return false;
    }
    cv::Rect bbox = cv::boundingRect(corners);
    cbRoi = cv::Rect(bbox.x - margin,
                     bbox.y - margin,
                     bbox.width + margin * 2,
                     bbox.height + margin * 2);

    qDebug() << "bbox: " << bbox.x << "," << bbox.y << ", " << bbox.width << ", " << bbox.height;

    return true;
}

QString OffsetCalibrationWorker::stateToString(OffsetCalibrationWorker::State state) {
    const QMetaEnum metaEnum = QMetaEnum::fromType<OffsetCalibrationWorker::State>();
    return QString::fromLatin1(metaEnum.valueToKey(static_cast<int>(state)));
}

void OffsetCalibrationWorker::repotFailure(const QString status){
    emit resultsReady(status);
    emit progressChanged(m_totalSteps, m_totalSteps);
    emit statusChanged(QString("Moving stage to zero position due to an error."));
    m_proxyService->setUVOff();
    m_state = State::Error;
    m_movingToZero = true;
    moveToZero();
}