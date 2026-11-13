#pragma once
#include "headers/calibrationStorage.h"
#include "BaseProcessor.h"
#include "OffsetCalibration/OffsetCalibrationParameters.h"

class CameraOffsetProcessor : public BaseProcessor {
    Q_OBJECT

    Q_PROPERTY(bool succeeded
                   READ succeeded
                       NOTIFY succeededChanged)

    Q_PROPERTY(QString calibrationStatusText
                   READ calibrationStatusText
                       NOTIFY calibrationStatusTextChanged)

    Q_PROPERTY(double progress
                   READ progress
                       NOTIFY progressChanged);

    Q_PROPERTY(QString statusMessage
                   READ statusMessage
                       WRITE setStatusMessage
                           NOTIFY statusMessageChanged)

public:
    explicit CameraOffsetProcessor(QString name,
          std::shared_ptr<ILoggerService> logger,
          std::shared_ptr<IServerProxyService> proxyService,
          std::shared_ptr<imaging::IImageService>imageService,
          std::shared_ptr<IStepperControllerService>stepperService,
          const OffsetCalibrationParameters defaultParams,
          QObject* parent=nullptr);

    Q_INVOKABLE virtual void execute(const QString& action) override;
    Q_INVOKABLE virtual void save() override {}
    Q_INVOKABLE void resetProgress();

    enum class ResultStatus {
        NotRun,
        NFOVPass,
        NFOVFail,
        WFOVFail,
        Pass,
        Fail
    };
    Q_ENUM(ResultStatus)

    bool succeeded() const { return m_succeeded; }
    QString calibrationStatusText() const { return m_calibrationStatusText; }

    void startCalibration();
    double progress() const {
        return m_progress;
    }
    bool busy() const {
        return m_busy;
    }
    QString statusMessage() const {
        return m_statusMessage;
    }

    void setStatusMessage(const QString &statusMessage) {
        if (m_statusMessage != statusMessage) {
            m_statusMessage = statusMessage;
            emit setStatusMessage(m_statusMessage);
        }
    }

public slots:
    void onResultsReady( QString result);
    void onClearOverlays();

signals:
    void progressChanged();
    void busyChanged();
    void message(QString text, QColor color, QString level);
    void statusMessageChanged();

    void setSucceeded(const QString& message);
    void setFailed(const QString& message);
    void succeededChanged();

    void calibrationStatusChanged();
    void calibrationStatusTextChanged();

private:
    double m_progress{0.0};
    bool m_busy{false};
    QString m_statusMessage;
    bool m_succeeded = false;

    ResultStatus m_calibrationStatus;
    QString m_calibrationStatusText;

    bool m_isHWConfigured{false};

    std::pair<uint32_t, uint32_t> m_imageDimensions{2016, 1452};
    calibrationStorage m_calStore;
    OffsetCalibrationParameters m_defaultParams;
};
