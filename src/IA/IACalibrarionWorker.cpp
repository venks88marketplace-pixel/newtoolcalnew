#include <QThread>
#include "IACalibrationWorker.h"
#include <interfaces/IImageService.h>
#include "calibrationStorage.h"
#include "utils.h"
#include <iostream>
#include <fstream> // Required header for std::ofstream


const std::map<IACalibrationWorker::State, IACalibrationWorker::Handler>
    IACalibrationWorker::s_handlers = {
        {IACalibrationWorker::State::Idle, &IACalibrationWorker::idleImageHandler},
        {IACalibrationWorker::State::WaitingForCheckerboardImages,&IACalibrationWorker::checkerBoardImageHandler},
        {IACalibrationWorker::State::WaitingForUVOnBlankImages,&IACalibrationWorker::uvOnBlankImageHandler},
        {IACalibrationWorker::State::WaitingForDMDImages,&IACalibrationWorker::uvDotImageHandler},
        {IACalibrationWorker::State::Error,&IACalibrationWorker::errorImageHandler}
};

void IdleImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);
void CheckerBoardImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);
void UVOnBlankImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);
void UVDotImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);
void ErrorImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);

IACalibrationWorker::IACalibrationWorker(
    std::shared_ptr<IServerProxyService> proxyService,
    bool mockedHW,
    std::vector<double> zPositions,
    std::pair<double,double> checkerDimentions,
    cv::Size patternSize,
    std::string saveImageFolder,
    QObject* parent)
    : QObject(parent),
    m_proxyService(std::move(proxyService)),
    m_mockedHW{mockedHW},
    m_zPlanes{std::move(zPositions)},
    m_trainCheckerDimensions{checkerDimentions},
    m_patternSize{patternSize},
    m_analysisFolderBase{saveImageFolder} {

    m_clahe = cv::createCLAHE(4.0, cv::Size(3,3));
    qDebug() << "StereoCalibratorWorker CREATED";
}

void IACalibrationWorker::run(){

    qDebug() << "run entered:" << QThread::currentThread();
    m_imageIndex = 0;
    int leftOffsetX,leftOffsetY, rightOffsetX,rightOffsetY;
    calibrationStorage calStore;
    calStore.getOffsets(leftOffsetX, leftOffsetY, rightOffsetX, rightOffsetY);

    try{
        m_analysisFolder = m_analysisFolderBase / calibration::CurrentDateString();
        if (!std::filesystem::exists(m_analysisFolder))
        {
            std::filesystem::create_directories(m_analysisFolder);
        }
    }catch(std::filesystem::filesystem_error& ex){
        qCritical() << "Error creating image folder: " << m_analysisFolder.string() << " " << ex.what();
    }

    m_iaCalibrator = std::make_unique<IACalibrator>(m_patternSize, m_analysisFolder, this);

    m_totalSteps = 6;

    if (!m_mockedHW) {
        m_proxyService->setCameraGainsAndExposure( m_leftGain, m_rightGain, m_exposureUsec);
        m_proxyService->setCameraROIs( m_imageDimensions.first, m_imageDimensions.second, leftOffsetX, leftOffsetY, rightOffsetX, rightOffsetY);
        m_proxyService->setNIR(m_nirSettings.top_intensity_percent, m_nirSettings.bottom_intensity_percent);
        m_proxyService->setCameraTrigger(true, m_triggerSettings.frequency_hz, m_triggerSettings.duration_ms);
        m_proxyService->setUVOff();

        emit statusChanged(QString("Homing XYZ motors..."));
        m_proxyService->homeStageMotors(); //xyz motors
    } else {
        onXYZHomeMoveCompleted();
    }
}

void IACalibrationWorker::startStep() {
    emit statusChanged(QString("Moving motor to position %1 mm").arg(m_zPlanes[0]));
    emit moveStepperRequested(m_zPlanes[0]);
}

void IACalibrationWorker::moveToZero() {
    m_movingToZero = true;

    emit statusChanged(QString("Moving motor to position 0 mm"));
    emit moveStepperRequested(0);
}

void IACalibrationWorker::onStepperMoveCompleted() {
    qDebug() << "stepper move completed";
    if (m_movingToZero) {
        m_current++;
        emit progressChanged(m_current, m_totalSteps);
        emit statusChanged(QString("Calibration Completed."));
        emit resultsReady(m_state != State::Error);
        emit finished(true);
    }else {
        emit acquireImagePairRequested(IImageService::Caller::IACal,  m_imageIndex++);
        m_state = State::WaitingForCheckerboardImages;
    }
}

void IACalibrationWorker::onXYZHomeMoveCompleted() {
    qDebug() << "Head home completed";

    emit statusChanged(QString("Homing Thorlabs stepper motor..."));
    emit progressChanged(m_current,  m_totalSteps);
    emit homeStepperRequested();
}

void IACalibrationWorker::onHomeMoveCompleted() {
    qDebug() << "Homing stepper move completed";
    m_current++;
    emit progressChanged(m_current, m_totalSteps);
    emit statusChanged(QString("Home completed"));
    startStep();
}

void IACalibrationWorker::onImagesReady(QPixmap imageL, QPixmap imageR) {

    qDebug() << "image ready received";
    cv::Mat limage = calibration::QPixmapToGrayMat(imageL);
    cv::Mat rimage = calibration::QPixmapToGrayMat(imageR);
    if (limage.rows == 0 || rimage.rows == 0) {
        qDebug() << "invalid images";
        m_state = State::Error;
    }

    try{
        std::string filename;
        if (m_state == State::WaitingForCheckerboardImages) {
            filename = "checkers";
        }
        else if (m_state == State::WaitingForUVOnBlankImages) {
            filename = "blank";
        } else {
            filename = "dot" ;
        }

        std::filesystem::path leftFname = m_analysisFolder / (filename + "-left.png");
        std::filesystem::path rightFname = m_analysisFolder / (filename + "-right.png");

        bool saveResult;
        saveResult = cv::imwrite(leftFname.string(), limage);
        if(!saveResult)
            qCritical() << "storing left image failed";
        saveResult = cv::imwrite(rightFname.string(), rimage);
        if(!saveResult)
            qCritical() << "storing right image failed";

        std::ofstream indexFile(m_analysisFolder / "ImageList3D.csv", std::ios::app);
        indexFile << m_imageIndex << "," << filename << std::endl;
        indexFile.close();

    }catch(...)
    {
        qCritical() << "Error saving files";
    }

    (this->*s_handlers.at(m_state))(limage, rimage);

    if (m_state == State::Error)
    {
        m_proxyService->setUVOff();
        emit statusChanged(QString("Error: Moving to zero"));
        moveToZero();
    }

    m_current++;
    emit progressChanged(m_current,  m_totalSteps);
}

void IACalibrationWorker::idleImageHandler(const cv::Mat& imageL, const cv::Mat& imageR) {

}
void IACalibrationWorker::errorImageHandler(const cv::Mat& imageL, const cv::Mat& imageR) {

}

void IACalibrationWorker::checkerBoardImageHandler(const cv::Mat& imageL, const cv::Mat& imageR) {
    emit statusChanged(QString("Processing images for position: %1").arg(m_zPlanes[m_index]));
    cv::Mat left,right;
    //improve contrast
    m_clahe->apply(imageL, left);
    m_clahe->apply(imageR, right);

    if (m_iaCalibrator->getCheckerboardCorners(left, right)){

        QImage blank(1024, 768, QImage::Format_Grayscale8);
        blank.fill(0);

        m_proxyService->setUV(6000, 600000);
        m_state = State::WaitingForUVOnBlankImages;
        m_proxyService->setDmdImage(blank);
        m_proxyService->setNIR(0,0);
        QThread::sleep(1);

        emit acquireImagePairRequested(IImageService::Caller::IACal,  m_imageIndex++);
    } else {
        qDebug() << "detected 0 points on checkerboard";
        m_proxyService->setUVOff();
        m_state = State::Error;
        emit statusChanged(QString("Moving to zero"));
        moveToZero();
    }
}

void IACalibrationWorker::uvOnBlankImageHandler(const cv::Mat& imageL, const cv::Mat& imageR) {
    m_leftNoUVImg = std::move(imageL);
    m_rightNoUVImg = std::move(imageR);

    auto img = m_iaCalibrator->createImage(5);

    m_state = State::WaitingForDMDImages;
    m_proxyService->setDmdImage(img);
    QThread::sleep(1);

    emit acquireImagePairRequested(IImageService::Caller::IACal,  m_imageIndex++);
}

void IACalibrationWorker::uvDotImageHandler(const cv::Mat& imageL, const cv::Mat& imageR) {
    cv::Mat subtractedLeft, subtractedRight;
    cv::subtract(imageL, m_leftNoUVImg, subtractedLeft);
    cv::subtract(imageR, m_rightNoUVImg, subtractedRight);

    auto left = calibration::stretch(subtractedLeft);
    auto right = calibration::stretch(subtractedRight);

    //cv::imshow("UV sutraction left", left);
    //cv::waitKey(0);

    //cv::imshow("UV sutraction right", right);
    //cv::waitKey(0);

    if(!m_iaCalibrator->trainIA(left, right)) {
        qDebug() << "error training IA axis";
        m_proxyService->setUVOff();
        m_state = State::Error;
        emit statusChanged(QString("Moving to zero"));
        moveToZero();
    } else {
        m_proxyService->setUVOff();
        m_state = State::Idle;
        emit statusChanged(QString("Moving to zero"));
        moveToZero();
    }
}



