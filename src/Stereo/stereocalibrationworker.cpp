#include "stereocalibrationworker.h"
#include <interfaces/IImageService.h>
#include "headers/calibrationStorage.h"
#include <iostream>
#include <fstream>
using namespace sps;

StereoCalibrationWorker::StereoCalibrationWorker(
        std::shared_ptr<IServerProxyService> proxyService,
        bool mockedHW,
        std::vector<double> zPositions,
        std::pair<double,double> checkerDimentions,
        cv::Size patternSize,
        const std::filesystem::path& analysisFolder,
        bool verifyMode,
        QObject* parent)
        : QObject(parent),
        m_proxyService(std::move(proxyService)),
        m_mockedHW{mockedHW},
        m_zPlanes{std::move(zPositions)},
        m_verifyMode{verifyMode},
        m_trainCheckerDimensions{checkerDimentions},
        m_analysisFolderBase{analysisFolder} {
    m_stereoCalibrator = std::make_unique<StereoCameraCalibrator>(patternSize, parent);
    m_clahe = cv::createCLAHE(4.0, cv::Size(3,3));
    qDebug() << "StereoCalibratorWorker CREATED";
}

void StereoCalibrationWorker::run(){

    qDebug() << "run entered:"
             << QThread::currentThread();

    int exposure;
    double leftGain, rightGain;
    int leftOffsetX,leftOffsetY, rightOffsetX,rightOffsetY;

    try{
        m_analysisFolder = m_analysisFolderBase / calibration::CurrentDateString();
        if (!std::filesystem::exists(m_analysisFolder))
        {
            std::filesystem::create_directories(m_analysisFolder / PLANES_FOLDER);
        }
    }catch(std::filesystem::filesystem_error& ex){
        qCritical() << "Error creating image folder: " << m_analysisFolder.string() << " " << ex.what();
    }

    m_calStore.getPhotometricParameters(exposure, leftGain, rightGain);
    m_calStore.getOffsets(leftOffsetX, leftOffsetY, rightOffsetX, rightOffsetY);
    if (!m_mockedHW) {
        m_proxyService->setCameraROIs( m_imageDimensions.first, m_imageDimensions.second, leftOffsetX, leftOffsetY, rightOffsetX, rightOffsetY);
        m_proxyService->setNIR(m_nirSettings.top_intensity_percent, m_nirSettings.bottom_intensity_percent);
        m_proxyService->setCameraGainsAndExposure(m_leftGain, m_rightGain, m_exposure);
        m_proxyService->setCameraTrigger(true, m_triggerSettings.frequency_hz, m_triggerSettings.duration_ms);
        emit statusChanged(QString("Homing XYZ motors..."));
        m_proxyService->homeStageMotors(); //xyz motors
        //need to pause here for the motors to settle
        QThread::sleep(8);
    }

    m_totalSteps = m_zPlanes.size() + (m_verifyMode ? 2 : 3);
    emit statusChanged(QString("Homing Thorlabs stepper motor..."));
    emit progressChanged(m_current,  m_totalSteps);
    emit homeStepperRequested();
}

void StereoCalibrationWorker::startStep() {
   emit statusChanged(QString("Moving motor to position %1 mm").arg(m_zPlanes[m_index]));
   emit moveStepperRequested(m_zPlanes[m_index]);
}

void StereoCalibrationWorker::moveToZero() {
    m_movingToZero = true;

    emit statusChanged(QString("Moving motor to position 0 mm"));
    emit moveStepperRequested(0);
}

void StereoCalibrationWorker::onStepperMoveCompleted() {
    qDebug() << "stepper move completed";
    if (m_movingToZero) {
        m_current++;
        emit progressChanged(m_current, m_totalSteps);
        emit statusChanged(QString(m_verifyMode ? "Verification completed." : "Calibration Completed."));
        emit resultsReady(std::move(m_stereoCalibrator->getResults()));
        emit finished(true);
    }else {
        emit acquireImagePairRequested(m_verifyMode ? IImageService::Caller::StereoVerification : IImageService::Caller::StereoCal,  m_index);
    }
}

void StereoCalibrationWorker::onHomeMoveCompleted() {
    qDebug() << "Homing stepper move completed";
    m_current++;
    emit progressChanged(m_current, m_totalSteps);
    emit statusChanged(QString("Home completed"));
    startStep();
}


void StereoCalibrationWorker::verificationHandler(cv::Mat& left, cv::Mat& right) {
    m_stereoCalibrator->verification(m_zPlanes[m_index], left, right,  m_analysisFolder);

    if (++m_index < m_zPlanes.size()){
        startStep();
    }
    else{
        m_stereoCalibrator->getResults().saveToFile(QString((m_analysisFolder / PLANES_FOLDER / "results.txt").c_str()));
        emit statusChanged(QString("Moving to zero"));
        moveToZero();
    }
}


void StereoCalibrationWorker::collectDataHandler(cv::Mat& left, cv::Mat& right)
{
    combo = m_stereoCalibrator->dataCollection(
        left,
        right,
        m_analysisFolder / PLANES_FOLDER,
        m_zPlanes[m_index]);

    if (++m_index < m_zPlanes.size())
        startStep();
    else{

        //train 3D
        emit statusChanged(QString("Training..."));
        std::cout << "CB dimensions: x,y: " << m_trainCheckerDimensions.first << "," << m_trainCheckerDimensions.second << std::endl;
        std::cout << "saved image folder: " << m_analysisFolder << std::endl;
        m_stereoCalibrator->train3D(m_trainCheckerDimensions.first,
                                    m_trainCheckerDimensions.second,
                                    m_analysisFolder / PLANES_FOLDER);

        m_current++;
        emit progressChanged(m_current, m_totalSteps);

        emit statusChanged(QString("Saving calibration data to file."));
        m_calStore.updateNfovStereo(m_stereoCalibrator->getParams().K1, m_stereoCalibrator->getParams().D1,
                                    m_stereoCalibrator->getParams().K2, m_stereoCalibrator->getParams().D2,
                                    m_stereoCalibrator->getParams().F, m_stereoCalibrator->getParams().E,
                                    m_stereoCalibrator->getParams().R, m_stereoCalibrator->getParams().T,
                                    m_stereoCalibrator->getParams().R1, m_stereoCalibrator->getParams().P1,
                                    m_stereoCalibrator->getParams().R2, m_stereoCalibrator->getParams().P2,
                                    m_stereoCalibrator->getParams().Q );
        m_calStore.updateNfovProjectionMatrices(m_stereoCalibrator->getParams().Pex1,
                                                m_stereoCalibrator->getParams().Pex2);

        emit statusChanged(QString("Moving to zero"));
        moveToZero();
    }
}

void StereoCalibrationWorker::onImagesReady(QPixmap imageL, QPixmap imageR) {
    qDebug() << "image ready received";
    emit statusChanged(QString("Processing images for position: %1").arg(m_zPlanes[m_index]));
    std::string fileNameBase = m_stereoCalibrator->currentDateString();

    cv::Mat limage = calibration::QPixmapToGrayMat(imageL);
    cv::Mat rimage = calibration::QPixmapToGrayMat(imageR);

    try{
        std::ostringstream ss;
        ss << m_zPlanes[m_index];

        std::filesystem::path leftFname = m_analysisFolder / (ss.str() + "-left.png");
        std::filesystem::path rightFname = m_analysisFolder / (ss.str() + "-right.png");

        bool saveResult;
        saveResult = cv::imwrite(leftFname.string(), limage);
        if(!saveResult)
            qCritical() << "storing left image failed";
        saveResult = cv::imwrite(rightFname.string(), rimage);
        if(!saveResult)
            qCritical() << "storing right image failed";

        std::ofstream indexFile(m_analysisFolder / "ImageList3D.csv", std::ios::app);
        indexFile << m_index << "," << ss.str() << std::endl;
        indexFile.close();

    }catch(...)
    {
        qCritical() << "Error saving files";
    }

    cv::Mat left,right;

    //improve contrast
    m_clahe->apply(limage, left);
    m_clahe->apply(rimage, right);

    if (left.rows == 0 || right.rows == 0) {
        qDebug() << "invalid images";
        return;
    }
    m_current++;
    emit progressChanged(m_current,  m_totalSteps);

    if(m_verifyMode){
        verificationHandler(left, right);
    } else {
        collectDataHandler(left, right);
    }
}

void StereoCalibrationWorker::updateCalFile(){

}

bool StereoCalibrationWorker::qualifyImagePair(const cv::Mat& l, const cv::Mat& r) {
    return true;
}

void StereoCalibrationWorker::storeImagePair( const std::string& path ) {

}
