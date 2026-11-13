#pragma once

#include <QObject>
#include <memory>
#include <string>
#include <thread>
#include <QList>
#include <vector>

#include "../services/LoggerService.h"
#include "../services/networkservice.h"
#include "../services/OphirMeterService.h"
#include "../services/uvsettingsservice.h"

struct CalibrationPoint {
    uint32_t target_irradiance_mw_per_cm2;
    uint32_t dac_count;
    uint32_t pd1_count;
    uint32_t pd2_count;
    uint32_t pd3_count;
    uint32_t current_count;
    double measured_irradiance;
    bool success;
    std::string error_message;
};

enum class CalibrationMode {
    REAL
};

class UVCalibrationProcessor : public QObject {
    Q_OBJECT

public:
    explicit UVCalibrationProcessor(const QString& name,
                                    std::shared_ptr<LoggerService> logger,
                                    std::shared_ptr<INetworkService> network,
                                    std::shared_ptr<IUVSettingsService> uvSettings,
                                    QObject* parent = nullptr);
    ~UVCalibrationProcessor();

    Q_PROPERTY(bool isConnected READ isConnected NOTIFY connectedChanged)
    Q_PROPERTY(double currentReading READ currentReading NOTIFY readingUpdated)
    Q_PROPERTY(bool isCalibrating READ isCalibrating NOTIFY calibratingChanged)
    Q_PROPERTY(int calibrationProgress READ calibrationProgress NOTIFY progressUpdated)
    Q_PROPERTY(bool serverConnected READ isServerConnected NOTIFY serverConnectedChanged)

    bool isConnected() const { return connected_; }
    double currentReading() const { return currentReading_; }
    bool isCalibrating() const { return isCalibrating_; }
    int calibrationProgress() const { return calibrationProgress_; }
    bool isServerConnected() const { return true; }

    Q_INVOKABLE void connectToMeter();
    Q_INVOKABLE void disconnectFromMeter();
    Q_INVOKABLE void startCalibration(int mode = 0);
    Q_INVOKABLE void stopCalibration();
    Q_INVOKABLE bool setFiveMmCircleDmd();

signals:
    void connectedChanged(bool connected);
    void readingUpdated();
    void calibratingChanged();
    void progressUpdated();
    void serverConnectedChanged(bool connected);
    void logMessage(const QString& message);
    void calibrationComplete();
    void calibrationError(const QString& error);
    void statusChanged();
    void calibrationPointAdded(uint32_t irradiance, uint32_t dac, double measured);

private:
    void doLogMessage(const QString& message);
    void runCalibrationLoopReal();
    CalibrationPoint findDacForIrradianceReal(uint32_t targetIrradiance, uint32_t startingDac);
    double readAveragedIrradiance(int numSamples);
    bool setDacAndWait(uint32_t dac, int waitMs);

    bool sendFiveMmCircleDmdFrame(QString& errorMessage);
    QByteArray buildFiveMmCircleFrame() const;

    std::vector<uint32_t> generateSequence(uint32_t start, uint32_t step, size_t count);

    QString name_;
    std::shared_ptr<LoggerService> logger_;
    std::shared_ptr<INetworkService> networkService_;
    std::shared_ptr<IUVSettingsService> uvSettingsService_;
    std::unique_ptr<OphirMeterService> ophirMeter_;

    bool connected_;
    double currentReading_;
    bool isCalibrating_;
    int calibrationProgress_;
    bool shouldStopCalibration_;
    std::thread calibrationThread_;
    QString statusMessage_;
    QList<CalibrationPoint> calibrationResults_;
    CalibrationMode calibrationMode_;

    static constexpr double TOLERANCE_PERCENT = 1.5;
    static constexpr uint32_t DAC_COARSE_STEP = 900;
    static constexpr uint32_t DAC_FINE_STEP = 100;
    static constexpr uint32_t MAX_DAC = 50000;
    static constexpr uint32_t MAX_COARSE_ATTEMPTS = 30;
    static constexpr uint32_t MAX_FINE_ATTEMPTS = 20;
    static constexpr uint32_t STABILIZATION_TIME_MS = 3000;
    static constexpr int SAMPLES_TO_AVERAGE = 3;
    static constexpr uint32_t PD1_FIXED_VALUE = 100;

    static constexpr double PREDICTIVE_CONSERVATIVE_FACTOR = 0.90;
    static constexpr uint32_t PREDICTIVE_MAX_JUMP_DAC = 3000;
    static constexpr int READ_RETRY_COUNT = 2;
    static constexpr uint32_t PREDICTIVE_MIN_JUMP_DAC = 100;

    void sendBatchedCalibrationResults();
};