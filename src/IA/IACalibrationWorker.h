#pragma once
#include <QObject>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include "../interfaces/IImageService.h"
#include "../interfaces/IServerProxyService.h"
#include "IA/IACalibrator.h"


using namespace imaging;

class IACalibrationWorker : public QObject
{
    Q_OBJECT

public:
    IACalibrationWorker(
        std::shared_ptr<IServerProxyService> proxyService,
        bool mockedHW,
        std::vector<double> zpositions,
        std::pair<double,double> checkerDimentions,
        cv::Size patternSize,
        std::string savedImageFolder,
        QObject* parent = nullptr);

    void updateCalFile();
    bool qualifyImagePair(const cv::Mat& , const cv::Mat& );
    void storeImagePair( const std::string& path );

    QJsonObject getZpositionsObject( const std::string& calConfigParams );

    enum class State
    {
        Idle,
        WaitingForCheckerboardImages,
        WaitingForUVOnBlankImages,
        WaitingForDMDImages,
        Error
    };

public slots:
    void run();
    void onImagesReady(QPixmap imageL, QPixmap imageR);
    void onStepperMoveCompleted();
    void onHomeMoveCompleted();
    void onXYZHomeMoveCompleted();

signals:
    void acquireImagePairRequested(IImageService::Caller, int);
    void moveStepperRequested(double);
    void homeStepperRequested();

    void message(QString text, QColor color, QString level);
    void progressChanged(int current, int total);
    void finished(bool success = false);
    void failed(QString text);
    void statusChanged(QString text);
    void resultsReady(bool success);

private:

    using Handler = void (IACalibrationWorker::*)(const cv::Mat& imageL,
                                                 const cv::Mat& imageR);

    static const std::map<State, Handler> s_handlers;

    std::shared_ptr<IServerProxyService> m_proxyService;
    std::unique_ptr<IACalibrator> m_iaCalibrator;
    int m_current{0};
    cv::Mat m_leftNoUVImg, m_rightNoUVImg;
    int m_index{0};
    int m_imageIndex{0};
    std::vector<double> m_zPlanes;
    int m_totalSteps;
    State m_state{State::Idle};

    std::filesystem::path m_analysisFolderBase;
    std::filesystem::path m_analysisFolder;
    cv::Size m_patternSize;
    std::pair<double,double> m_trainCheckerDimensions;

    bool m_movingToZero{false};
    bool m_mockedHW;
    int m_checkPointIndex;
    cv::Ptr<cv::CLAHE> m_clahe;

    const double m_leftGain{7.0}, m_rightGain{7.0};
    const double m_exposureUsec{5000.0};
    const nirLedControls m_nirSettings{30, 30};
    const cameraTrigger m_triggerSettings{60,6};
    const std::pair<uint32_t, uint32_t> m_imageDimensions{2016, 1452};

    void idleImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);
    void checkerBoardImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);
    void uvOnBlankImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);
    void uvDotImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);
    void errorImageHandler(const cv::Mat& imageL, const cv::Mat& imageR);

    void getCheckerboardCorners(const cv::Mat& img, std::vector<cv::Point2f>& corners);

    void startStep();
    void moveToZero();

};

