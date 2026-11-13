#include "CameraOffsetProcessor.h"
#include <QDir>
#include "src/OffsetCalibration/offsetcalibrationworker.h"

using imaging::IImageService;

CameraOffsetProcessor::CameraOffsetProcessor(QString name,
           std::shared_ptr<ILoggerService> logger,
           std::shared_ptr<IServerProxyService> proxy,
           std::shared_ptr<IImageService> imageprovider,
           std::shared_ptr<IStepperControllerService> stepperService,
           const OffsetCalibrationParameters defaultParams,
           QObject* parent)
    : BaseProcessor(name, std::move(logger),
                    std::move(proxy),
                    std::move(imageprovider),
                    std::move(stepperService),
                    parent),
                    m_defaultParams(defaultParams){
    m_leftOverlay = new ShapeModel(this);
    m_rightOverlay = new ShapeModel(this);
    m_wideOverlay = new ShapeModel(this);

    qDebug() << "CameraOffsetProcessor CREATED";
}

void CameraOffsetProcessor::startCalibration(){

    auto* thread = new QThread(this);
    auto* worker = new OffsetCalibrationWorker(m_proxy,
                                              IsImageServiceSimulated(),
                                              uiMessage(),
                                              leftOverlay(),
                                              rightOverlay(),
                                              wideOverlay(),
                                              m_defaultParams);
    worker->moveToThread(thread);

    connect(thread,
            &QThread::started,
            worker,
            &OffsetCalibrationWorker::run);
    connect(worker,
            &OffsetCalibrationWorker::message,
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
        &OffsetCalibrationWorker::finished,
        this,
        [this](bool success) {
            m_busy = false;
            if (success)
                m_progress = 1.0;
            emit progressChanged();
            emit busyChanged();
        });
    connect(worker,
            &OffsetCalibrationWorker::finished,
            thread,
            &QThread::quit);
    connect(worker,
            &OffsetCalibrationWorker::finished,
            worker,
            &QObject::deleteLater);
    connect(thread,
            &QThread::finished,
            thread,
            &QObject::deleteLater);
    connect(
        worker,
        &OffsetCalibrationWorker::progressChanged,
        this,
        [this](int current, int total){
            if (total <= 0)
                m_progress = 0.0;
            else
                m_progress =
                    static_cast<double>(current) /
                    static_cast<double>(total);
            emit progressChanged();
        });

    connect(
        worker,
        &OffsetCalibrationWorker::failed,
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
            &OffsetCalibrationWorker::statusChanged,
            this,
            [this](const QString& text)
            {
                if (m_statusMessage == text)
                    return;
                m_statusMessage = text;
                emit statusMessageChanged();
            });

    connect(worker,
            &OffsetCalibrationWorker::acquireImagePairRequested,
            m_imageService.get(),
            &IImageService::getImagePair,
            Qt::QueuedConnection);

    connect(worker,
            &OffsetCalibrationWorker::acquireWideImageRequested,
            m_imageService.get(),
            &IImageService::getWideImage,
            Qt::QueuedConnection);

    connect(m_imageService.get(),
            &IImageService::imagesReady,
            worker,
            &OffsetCalibrationWorker::onImagesReady,
            Qt::QueuedConnection);

    connect(m_imageService.get(),
            &IImageService::wideFramePixmapReady,
            worker,
            &OffsetCalibrationWorker::onWideFramePixmapReady,
            Qt::QueuedConnection);

    connect(worker,
            &OffsetCalibrationWorker::moveStepperRequested,
            m_stepperService.get(),
            &IStepperControllerService::moveToPosition,
            Qt::QueuedConnection);

    connect( m_stepperService.get(),
            &IStepperControllerService::moveCompleted,
            worker,
            &OffsetCalibrationWorker::onStepperMoveCompleted,
            Qt::QueuedConnection);

    connect(worker,
            &OffsetCalibrationWorker::homeStepperRequested,
            m_stepperService.get(),
            &IStepperControllerService::moveHome,
            Qt::QueuedConnection);

    connect( m_stepperService.get(),
            &IStepperControllerService::moveHomeCompleted,
            worker,
            &OffsetCalibrationWorker::onHomeMoveCompleted,
            Qt::QueuedConnection);

    connect(worker,
            &OffsetCalibrationWorker::resultsReady,
            this,
            &CameraOffsetProcessor::onResultsReady,
            Qt::QueuedConnection);

    connect(worker,
            &OffsetCalibrationWorker::clearNfovImages,
            this,
            [this](const QPixmap& image)
            {
                m_imageService->clearNfovImages(image);
            });

    qDebug() << "ImageService thread:"
             << m_imageService->thread();

    qDebug() << "Worker affinity:"
             << worker->thread();

    thread->start();
}

void CameraOffsetProcessor::execute(const QString& action) {
    resetProgress();
    emit calibrationStatusTextChanged();

    startCalibration();
}

void CameraOffsetProcessor::onResultsReady(QString result) {
    m_calibrationStatusText = result;
    emit calibrationStatusTextChanged();
}

void CameraOffsetProcessor::resetProgress() {
    m_progress = 0.0;
    emit progressChanged();
}

void CameraOffsetProcessor::onClearOverlays() {
    clearOverlays();
}
