#include "IACalibrationProcessor.h"
#include "interfaces/IStepperControllerService.h"
#include "src/AppConfig.h"
#include <QTimer>
#include "IA/IACalibrationWorker.h"

using imaging::IImageService;

 IACalibrationProcessor::IACalibrationProcessor(QString name,
                                    std::shared_ptr<ILoggerService> logger,
                                    std::shared_ptr<IServerProxyService> proxy,
                                    std::shared_ptr<IImageService> imageService,
                                    std::shared_ptr<IStepperControllerService> stepperService,
                                    std::vector<double> zPositions,
                                    std::pair<double,double> checkerDimentions,
                                    cv::Size patternSize,
                                    QObject* parent)
    : BaseProcessor(name, std::move(logger),
                    std::move(proxy),
                    std::move(imageService),
                    std::move(stepperService),
                    parent),
                    m_zPositions{std::move(zPositions)},
                    m_trainCheckerDimensions{std::move(checkerDimentions)},
                    m_patternSize{std::move(patternSize)} {
    m_leftOverlay = new ShapeModel(this);
    m_rightOverlay = new ShapeModel(this);
    qDebug() << "IACalibrationProcessor CREATED";
    qDebug() << "IACalibrationProcessor thread =" << thread();
 }

 void IACalibrationProcessor::configure(const QJsonObject& settings){
     m_analysisFolder = settings.value("analysis_folder").toString("").toStdString();
 }

 void IACalibrationProcessor::execute(const QString& action) {
     //hardware setup
     resetProgress();
     m_calibrationStatus = ResultStatus::NotRun;
     emit calibrationStatusChanged();
     emit calibrationStatusTextChanged();
     startCalibration();
 }

 void IACalibrationProcessor::startCalibration(){

     auto* thread = new QThread(this);
     auto* worker = new IACalibrationWorker(m_proxy,
                                       IsImageServiceSimulated(),
                                       m_zPositions,
                                       m_trainCheckerDimensions,
                                       m_patternSize,
                                       m_analysisFolder );

     worker->moveToThread(thread);

     connect(thread,
             &QThread::started,
             worker,
             &IACalibrationWorker::run);
     connect(worker,
             &IACalibrationWorker::message,
             this,
             [this](const QString& text,
                    QColor color,
                    QString level) {
                 m_uiMessage->addMessage(
                     text,
                     color,
                     "INFO");
             });
     connect(
         worker,
         &IACalibrationWorker::finished,
         this,
         [this](bool success) {
             m_busy = false;
             if (success)
                m_progress = 1.0;
             emit progressChanged(m_progress);
             emit busyChanged();
         });
     connect(worker,
             &IACalibrationWorker::finished,
             thread,
             &QThread::quit);
     connect(worker,
             &IACalibrationWorker::finished,
             worker,
             &QObject::deleteLater);
     connect(thread,
             &QThread::finished,
             thread,
             &QObject::deleteLater);
     connect(
         worker,
         &IACalibrationWorker::progressChanged,
         this,
         [this](int current, int total){
             if (total <= 0)
                 m_progress = 0.0;
             else
                 m_progress =
                     static_cast<double>(current) /
                     static_cast<double>(total);
             emit progressChanged(m_progress);
         });

     connect(
         worker,
         &IACalibrationWorker::failed,
         this,
         [this](const QString& error) {
             m_busy = false;
             m_uiMessage->addMessage(
                 error,
                 Qt::red,
                 "ERROR");
             emit busyChanged();
         });

     connect(worker,
             &IACalibrationWorker::statusChanged,
             this,
             [this](const QString& text)
             {
                 setStatusMessage(text);
             },
             Qt::QueuedConnection);

     connect(worker,
             &IACalibrationWorker::acquireImagePairRequested,
             m_imageService.get(),
             &IImageService::getImagePair,
             Qt::QueuedConnection);

     connect(m_imageService.get(),
             &IImageService::imagesReady,
             worker,
             &IACalibrationWorker::onImagesReady,
             Qt::QueuedConnection);

     connect(m_imageService.get(),
             &IImageService::imagesReady,
             m_imageService.get(),
             [](QPixmap imageL, QPixmap imageR)
             {
                 qDebug() << "TEST: imagesReady received in main thread"
                          << imageL.size()
                          << imageR.size();
             });

     connect(m_imageService.get(),
             &IImageService::imagesReady,
             worker,
             [](QPixmap imageL, QPixmap imageR)
             {
                 qDebug() << "TEST: imagesReady arrived in worker thread"
                          << QThread::currentThread()
                          << imageL.size()
                          << imageR.size();
             },
             Qt::QueuedConnection);

     connect(worker,
             &IACalibrationWorker::moveStepperRequested,
             m_stepperService.get(),
             &IStepperControllerService::moveToPosition,
             Qt::QueuedConnection);

     connect( m_stepperService.get(),
             &IStepperControllerService::moveCompleted,
             worker,
             &IACalibrationWorker::onStepperMoveCompleted,
             Qt::QueuedConnection);

     connect(worker,
             &IACalibrationWorker::homeStepperRequested,
             m_stepperService.get(),
             &IStepperControllerService::moveHome,
             Qt::QueuedConnection);

     connect( m_stepperService.get(),
             &IStepperControllerService::moveHomeCompleted,
             worker,
             &IACalibrationWorker::onHomeMoveCompleted,
             Qt::QueuedConnection);

     connect( m_proxy.get(),
             &IServerProxyService::stageHomeCompleted,
             worker,
             &IACalibrationWorker::onXYZHomeMoveCompleted,
             Qt::QueuedConnection);

     connect(worker,
             &IACalibrationWorker::resultsReady,
             this,
             &IACalibrationProcessor::onResultsReady,
             Qt::QueuedConnection);


     qDebug() << "ImageService thread:"
              << m_imageService->thread();

     qDebug() << "Worker affinity:"
              << worker->thread();

     thread->start();
 }


void IACalibrationProcessor::onResultsReady(bool result) {
    m_passed = result;
    m_calibrationStatus = m_passed ? ResultStatus::Pass : ResultStatus::Fail;
    emit calibrationStatusTextChanged();
    emit passedChanged();
}

QString IACalibrationProcessor::calibrationStatusText() {
    switch (m_calibrationStatus) {
    case ResultStatus::NotRun:
        return "Not Run";
    case ResultStatus::Pass:
        return "Pass";
    case ResultStatus::Fail:
        return "Fail";
    }
    return {};
}

void IACalibrationProcessor::resetProgress() {
    m_progress = 0.0;
    emit progressChanged(m_progress);
}

