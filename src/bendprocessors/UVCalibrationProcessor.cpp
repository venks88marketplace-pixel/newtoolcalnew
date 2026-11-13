#include "UVCalibrationProcessor.h"
#include <QTimer>
#include <QThread>
#include <QDebug>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "neo-calibration-server-api/calibration_server_api.pb.h"

UVCalibrationProcessor::UVCalibrationProcessor(const QString& name, 
                                               std::shared_ptr<LoggerService> logger, 
                                               std::shared_ptr<INetworkService> network,
                                               std::shared_ptr<IUVSettingsService> uvSettings,
                                               QObject* parent)
    : QObject(parent),
      name_(name),
      logger_(logger),
      networkService_(network),
      uvSettingsService_(uvSettings),
      connected_(false),
      currentReading_(0.0),
      isCalibrating_(false),
      calibrationProgress_(0),
      shouldStopCalibration_(false),
      ophirMeter_(std::make_unique<OphirMeterService>()),
      calibrationMode_(CalibrationMode::REAL)
{
}

UVCalibrationProcessor::~UVCalibrationProcessor() {
    stopCalibration();
    if (ophirMeter_ && ophirMeter_->isConnected()) {
        ophirMeter_->disconnect();
    }
}

void UVCalibrationProcessor::connectToMeter() {
    if (!ophirMeter_) {
        emit calibrationError("Ophir Meter service not initialized");
        return;
    }

    if (ophirMeter_->connect()) {
        connected_ = true;
        statusMessage_ = "Connected to Ophir Meter";
        
        // Set sensor to 5mm circular area
        ophirMeter_->setSensorArea(0.1963);  // ~0.1963 cm² for 5mm diameter
        
        auto callback = [this](double irradiance) {
            QMetaObject::invokeMethod(this, [this, irradiance]() {
                currentReading_ = irradiance;
                emit readingUpdated();
            });
        };
        
        try {
            ophirMeter_->startStreaming(callback, 100);
            doLogMessage("Started streaming from Ophir Meter (5mm sensor area: 0.1963 cm²)");
        } catch (const std::exception& e) {
            doLogMessage("Failed to start streaming: " + QString::fromStdString(e.what()));
        }
    } else {
        connected_ = false;
        statusMessage_ = "Failed to connect: " + QString::fromStdString(ophirMeter_->getLastError());
        doLogMessage(statusMessage_);
    }
    
    emit connectedChanged(connected_);
    emit statusChanged();
}

void UVCalibrationProcessor::disconnectFromMeter() {
    stopCalibration();
    if (ophirMeter_) {
        ophirMeter_->stopStreaming();
        ophirMeter_->disconnect();
    }
    connected_ = false;
    currentReading_ = 0.0;
    statusMessage_ = "Disconnected from Ophir Meter";
    
    doLogMessage(statusMessage_);
    emit connectedChanged(false);
    emit statusChanged();
    emit readingUpdated();
}

void UVCalibrationProcessor::startCalibration(int mode) {
    if (!connected_) {
        emit calibrationError("Ophir Meter not connected");
        return;
    }

    if (isCalibrating_) {
        return;
    }

    if (!uvSettingsService_) {
        emit calibrationError("UV Settings Service not available");
        return;
    }

    QString dmdError;
    if (!setFiveMmCircleDmd()) {
        emit calibrationError("Failed to set DMD to 5mm circle: " + dmdError);
        return;
    }

    isCalibrating_ = true;
    shouldStopCalibration_ = false;
    calibrationProgress_ = 0;
    calibrationResults_.clear();
    calibrationMode_ = CalibrationMode::REAL;

    doLogMessage("=== Starting UV Calibration ===");
    doLogMessage("DMD set to 5mm circle before UV irradiation step.");

    calibrationThread_ = std::thread([this]() {
        runCalibrationLoopReal();
    });

    emit calibratingChanged();
    emit progressUpdated();
}

void UVCalibrationProcessor::stopCalibration() {
    if (isCalibrating_) {
        shouldStopCalibration_ = true;
        if (calibrationThread_.joinable()) {
            calibrationThread_.join();
        }
        
        // Turn off UV
        if (uvSettingsService_) {
            doLogMessage("Turning off UV...");
            uvSettingsService_->setEnabled(false);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        isCalibrating_ = false;
        emit calibratingChanged();
    }
}

bool UVCalibrationProcessor::setDacAndWait(uint32_t dac, int waitMs) {
    if (!uvSettingsService_) {
        return false;
    }
    
    uvSettingsService_->setPower(static_cast<int>(dac));
    uvSettingsService_->apply();
    std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
    
    return true;
}

// Generate an arithmetic sequence of unsigned ints
std::vector<uint32_t> UVCalibrationProcessor::generateSequence(uint32_t start, uint32_t step, size_t count) {
    std::vector<uint32_t> seq;
    if (count == 0 || step == 0) {
        return seq;
    }
    seq.reserve(count);
    uint32_t value = start;
    for (size_t i = 0; i < count; ++i) {
        seq.push_back(value);
        value += step;
    }
    return seq;
}

double UVCalibrationProcessor::readAveragedIrradiance(int numSamples) {
    double sum = 0.0;
    int successfulReads = 0;
    
    for (int i = 0; i < numSamples; ++i) {
        try {
            double reading = ophirMeter_->readIrradiance();
            sum += reading;
            successfulReads++;
        } catch (const std::exception& e) {
            // Silently ignore read errors
        }
        
        if (i < numSamples - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    if (successfulReads == 0) {
        return -1.0;
    }
    
    return sum / successfulReads;
}


void UVCalibrationProcessor::runCalibrationLoopReal()
{
    doLogMessage("\n=== REAL MODE CALIBRATION ===");

    std::vector<uint32_t> targetIrradianceValues;
    for (uint32_t i = 3; i <= 50; ++i) {
        targetIrradianceValues.push_back(i);
    }

    const int totalSteps = static_cast<int>(targetIrradianceValues.size());
    int currentStep = 0;

    const double tolerancePercent = TOLERANCE_PERCENT;
    const int stabilizationMs = static_cast<int>(STABILIZATION_TIME_MS);
    const int samplesPerReading = SAMPLES_TO_AVERAGE;

    // Allow enough attempts for a complete binary search over the DAC range.
    const int maxAttempts = std::max(
        16,
        static_cast<int>(MAX_COARSE_ATTEMPTS + MAX_FINE_ATTEMPTS)
    );

    // Known manual reference provided by Ross J:
    // DAC 7000 produced approximately 30 mW/cm².
    constexpr double REFERENCE_DAC = 7000.0;
    constexpr double REFERENCE_IRRADIANCE = 30.0;

    // Remember the last successful calibration point.
    uint32_t lastSuccessfulDac = 0;
    double lastSuccessfulMeasuredIrradiance = -1.0;

    auto clampDac = [](double value) -> uint32_t {
        value = std::max(1.0, std::min(value, static_cast<double>(MAX_DAC)));
        return static_cast<uint32_t>(std::lround(value));
    };

    for (uint32_t targetIrradiance : targetIrradianceValues) {

        if (shouldStopCalibration_) {
            doLogMessage("Calibration stopped by user");
            break;
        }

        currentStep++;

        calibrationProgress_ = (currentStep * 100) / totalSteps;

        QMetaObject::invokeMethod(this, [this]() {
            emit progressUpdated();
        });

        std::stringstream headerSs;
        headerSs << "\n=== Step " << currentStep << "/" << totalSteps
                 << ": Target Irradiance=" << targetIrradiance
                 << " mW/cm² ===";

        doLogMessage(QString::fromStdString(headerSs.str()));

        const double lowerBound =
            targetIrradiance * (1.0 - tolerancePercent / 100.0);

        const double upperBound =
            targetIrradiance * (1.0 + tolerancePercent / 100.0);

        std::stringstream boundSs;
        boundSs << "Tolerance bounds: "
                << std::fixed << std::setprecision(2)
                << lowerBound << " - " << upperBound
                << " mW/cm²";

        doLogMessage(QString::fromStdString(boundSs.str()));
        
        // Initial DAC estimate
        uint32_t currentDac = 0;

        if (lastSuccessfulMeasuredIrradiance > 0.0 &&
            lastSuccessfulDac > 0) {

            // Use the previous successful calibration point.
            // This should make consecutive targets require smaller
            // adjustments.
            const double ratio =
                static_cast<double>(targetIrradiance) /
                lastSuccessfulMeasuredIrradiance;

            const double estimatedDac =
                static_cast<double>(lastSuccessfulDac) * ratio;

            currentDac = clampDac(estimatedDac);
        }
        else {
            // First calibration point:
            // Use the known manual DAC/irradiance relationship.
            const double estimatedDac =
                REFERENCE_DAC *
                (static_cast<double>(targetIrradiance) /
                 REFERENCE_IRRADIANCE);

            currentDac = clampDac(estimatedDac);
        }

        std::stringstream startSs;
        startSs << "Starting DAC search"
                << " | Last successful DAC: " << lastSuccessfulDac
                << " | Initial DAC estimate: " << currentDac;

        doLogMessage(QString::fromStdString(startSs.str()));

        // Search bounds
        uint32_t lowDac = 0;
        uint32_t highDac = MAX_DAC;

        bool haveLow = false;
        bool haveHigh = false;

        bool foundTarget = false;

        int attempt = 0;
        int readRetriesRemaining = READ_RETRY_COUNT;


        // Calibration search


        while (!shouldStopCalibration_ &&
               !foundTarget &&
               attempt < maxAttempts) {

            attempt++;

            std::stringstream attemptSs;
            attemptSs << "\nAttempt " << attempt
                      << ": Setting DAC=" << currentDac;

            doLogMessage(QString::fromStdString(attemptSs.str()));

            // Turn UV on and apply the requested DAC.
            doLogMessage("Turning ON UV...");

            uvSettingsService_->setPower(static_cast<int>(currentDac));
            uvSettingsService_->setEnabled(true);
            uvSettingsService_->apply();

            doLogMessage(
                "Waiting " +
                QString::number(stabilizationMs / 1000) +
                " seconds for stabilization..."
            );

            std::this_thread::sleep_for(
                std::chrono::milliseconds(stabilizationMs)
            );

            // Read irradiance.
            doLogMessage("Reading irradiance from Ophir meter...");

            const double measuredIrradiance =
                readAveragedIrradiance(samplesPerReading);

            // Turn UV off after measurement.
            doLogMessage("Turning OFF UV");

            uvSettingsService_->setEnabled(false);

            std::this_thread::sleep_for(
                std::chrono::milliseconds(1000)
            );

            // Handle failed meter reads
            if (!std::isfinite(measuredIrradiance) ||
                measuredIrradiance < 0.0) {

                doLogMessage(
                    "ERROR: Failed to read valid irradiance "
                    "from Ophir meter"
                );

                if (readRetriesRemaining > 0) {
                    readRetriesRemaining--;

                    doLogMessage(
                        QString("Retrying read at same DAC "
                                "(retries left: %1)")
                            .arg(readRetriesRemaining)
                    );

                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(200)
                    );

                    continue;
                }

                readRetriesRemaining = READ_RETRY_COUNT;

                doLogMessage(
                    "Meter read retries exhausted. "
                    "Moving DAC slightly before next attempt."
                );

                currentDac = clampDac(
                    static_cast<double>(currentDac) + 100.0
                );

                continue;
            }

            readRetriesRemaining = READ_RETRY_COUNT;

            std::stringstream readSs;
            readSs << "Measured irradiance: "
                   << std::fixed << std::setprecision(3)
                   << measuredIrradiance << " mW/cm²";

            doLogMessage(QString::fromStdString(readSs.str()));

            // Check whether the target has been reached
            if (measuredIrradiance >= lowerBound &&
                measuredIrradiance <= upperBound) {

                std::stringstream successSs;
                successSs << "SUCCESS: Within tolerance!"
                          << " DAC=" << currentDac
                          << " | Irradiance="
                          << std::fixed << std::setprecision(3)
                          << measuredIrradiance << " mW/cm²";

                doLogMessage(QString::fromStdString(successSs.str()));

                CalibrationPoint point;

                point.target_irradiance_mw_per_cm2 = targetIrradiance;
                point.dac_count = currentDac;
                point.measured_irradiance = measuredIrradiance;
                point.pd1_count = PD1_FIXED_VALUE;
                point.pd2_count = 1000;
                point.pd3_count = 1000;
                point.current_count = 5000;
                point.success = true;

                calibrationResults_.append(point);

                // Save this successful point for the next target, checked on RB1 for faster setting
                lastSuccessfulDac = currentDac;
                lastSuccessfulMeasuredIrradiance = measuredIrradiance;

                QMetaObject::invokeMethod(
                    this,
                    [this, targetIrradiance, currentDac, measuredIrradiance]() {
                        emit calibrationPointAdded(
                            static_cast<uint32_t>(targetIrradiance),
                            currentDac,
                            measuredIrradiance
                        );
                    }
                );

                foundTarget = true;
                break;
            }

            // Update the search bounds
            if (measuredIrradiance < lowerBound) {

                // Current DAC is too low.
                lowDac = std::max(lowDac, currentDac);
                haveLow = true;

                std::stringstream belowSs;
                belowSs << "Below target. "
                        << "Measured=" << measuredIrradiance
                        << " | Target=" << targetIrradiance
                        << " | Low DAC=" << lowDac;

                doLogMessage(QString::fromStdString(belowSs.str()));
            }
            else {

                // Current DAC is too high.
                highDac = std::min(highDac, currentDac);
                haveHigh = true;

                std::stringstream aboveSs;
                aboveSs << "Above target. "
                        << "Measured=" << measuredIrradiance
                        << " | Target=" << targetIrradiance
                        << " | High DAC=" << highDac;

                doLogMessage(QString::fromStdString(aboveSs.str()));
            }

            // Choose the next DAC
            if (haveLow && haveHigh) {

                // Both sides of the target have now been measured.
                // This is a ideal bracket, so binary search is valid.

                if (highDac <= lowDac + 1) {
                    doLogMessage(
                        "Binary search bounds are adjacent. "
                        "No additional integer DAC value is available."
                    );

                    break;
                }

                const uint32_t nextDac =
                    lowDac + ((highDac - lowDac) / 2u);

                std::stringstream searchSs;
                searchSs << "Binary search:"
                         << " Low=" << lowDac
                         << " | High=" << highDac
                         << " | Next DAC=" << nextDac;

                doLogMessage(QString::fromStdString(searchSs.str()));

                currentDac = nextDac;
            }
            else {

                // Using proportional adjustment instead of binary search.

                const double ratio =
                    static_cast<double>(targetIrradiance) /
                    measuredIrradiance;

                // Limit each adjustment to 0.5x - 2.0x.
                // This prevents large jumps while finding the bracket.
                const double limitedRatio =
                    std::max(0.5, std::min(ratio, 2.0));

                double nextDac =
                    static_cast<double>(currentDac) * limitedRatio;

                // Ensuring the next DAC actually moves in the
                // required direction despite rounding.
                if (measuredIrradiance < lowerBound &&
                    nextDac <= currentDac) {

                    nextDac = static_cast<double>(currentDac) + 1.0;
                }
                else if (measuredIrradiance > upperBound &&
                         nextDac >= currentDac) {

                    nextDac = static_cast<double>(currentDac) - 1.0;
                }

                currentDac = clampDac(nextDac);

                std::stringstream adjustSs;
                adjustSs << "Proportional adjustment:"
                         << " Ratio=" << std::fixed << std::setprecision(3)
                         << ratio
                         << " | Next DAC=" << currentDac;

                doLogMessage(QString::fromStdString(adjustSs.str()));
            }
        }

        // Record unsuccessful calibration point
        if (!foundTarget) {

            std::stringstream failSs;
            failSs << "ERROR: Failed to find DAC for target "
                   << targetIrradiance
                   << " mW/cm² after "
                   << attempt << " attempts";

            doLogMessage(QString::fromStdString(failSs.str()));

            CalibrationPoint failPt;

            failPt.target_irradiance_mw_per_cm2 = targetIrradiance;
            failPt.dac_count = currentDac;
            failPt.measured_irradiance = -1.0;
            failPt.pd1_count = PD1_FIXED_VALUE;
            failPt.pd2_count = 1000;
            failPt.pd3_count = 1000;
            failPt.current_count = 5000;
            failPt.success = false;
            failPt.error_message = "Max attempts exceeded";

            calibrationResults_.append(failPt);

            QMetaObject::invokeMethod(
                this,
                [this, targetIrradiance]() {
                    emit calibrationPointAdded(
                        static_cast<uint32_t>(targetIrradiance),
                        0,
                        -1.0
                    );
                }
            );
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1000)
        );
    }

    // Upload successful calibration points to HBC via cal server.
    sendBatchedCalibrationResults();

    isCalibrating_ = false;
    calibrationProgress_ = 100;

    std::stringstream finalSs;
    finalSs << "\n=== REAL Calibration Complete ===\n"
            << "Total points collected: "
            << calibrationResults_.size();

    doLogMessage(QString::fromStdString(finalSs.str()));

    QMetaObject::invokeMethod(this, [this]() {
        emit calibratingChanged();
        emit progressUpdated();
        emit calibrationComplete();
    });
}


// Upload all successful calibration points to cal server
void UVCalibrationProcessor::sendBatchedCalibrationResults() {
    if (!networkService_ || !networkService_->isConnected()) {
        doLogMessage("Network not connected, skipping batched upload");
        return;
    }

    try {
        for (const CalibrationPoint& p : calibrationResults_) {
            if (!p.success) continue;

            CalibrationApi::ToServer toServer;
            auto* msg = toServer.mutable_start_calibration();
            msg->set_irradiance_mw_per_cm2(p.target_irradiance_mw_per_cm2);
            msg->set_dac_count(p.dac_count);

            QByteArray data(static_cast<int>(toServer.ByteSizeLong()), '\0');
            if (!toServer.SerializeToArray(data.data(), data.size())) {
                doLogMessage("ERROR: Failed to serialize calibration pair for batched upload");
                continue;
            }

            networkService_->sendMessage(data);

            std::stringstream ss;
            ss << "  ✓ Batched sent to server: Irradiance=" << std::fixed << std::setprecision(3)
               << p.target_irradiance_mw_per_cm2 << " mW/cm² | DAC=" << p.dac_count
               << " | Measured=" << p.measured_irradiance;
            doLogMessage(QString::fromStdString(ss.str()));
            // Safety sleep to avoid the timeout from server side.
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }
    } catch (const std::exception& e) {
        doLogMessage("ERROR: Batched upload exception: " + QString::fromStdString(e.what()));
    }
}

CalibrationPoint UVCalibrationProcessor::findDacForIrradianceReal(uint32_t targetIrradiance, uint32_t startingDac)
{
    // Simple wrapper that attempts to find DAC by running a small search.
    // The full search logic is implemented in runCalibrationLoopReal; this function provides
    // a stubbed point if you need to call it separately.
    CalibrationPoint result;
    result.target_irradiance_mw_per_cm2 = targetIrradiance;
    result.success = false;
    result.pd1_count = PD1_FIXED_VALUE;
    result.pd2_count = 1000;
    result.pd3_count = 1000;
    result.current_count = 5000;
    return result;
}

void UVCalibrationProcessor::doLogMessage(const QString& message) {
    qDebug() << message;
    emit logMessage(message);
}

QByteArray UVCalibrationProcessor::buildFiveMmCircleFrame() const
{
    const int width = 1024;
    const int height = 768;

    const int cx = width / 2;
    const int cy = height / 2;
    const int radius = 125; //5mm UV circle diameter.

    // Create a black grayscale image
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1);

    // Draw a filled white circle at the center
    cv::circle(
        image,
        cv::Point(cx, cy),
        radius,
        cv::Scalar(255),
        cv::FILLED,
        cv::LINE_AA
    );

    // Convert OpenCV image to QByteArray
    QByteArray byteArray;
    byteArray.append(
        reinterpret_cast<const char*>(image.data),
        static_cast<qsizetype>(image.total() * image.elemSize())
    );

    return byteArray;
}

bool UVCalibrationProcessor::setFiveMmCircleDmd() {
    QString error;
    return sendFiveMmCircleDmdFrame(error);
}

bool UVCalibrationProcessor::sendFiveMmCircleDmdFrame(QString& errorMessage) {
    if (!networkService_ || !networkService_->isConnected()) {
        errorMessage = "Network not connected; cannot send DMD circle.";
        doLogMessage(errorMessage);
        return false;
    }

    try {
        QByteArray frame = buildFiveMmCircleFrame();

        CalibrationApi::ToServer toServer;
        auto* dmd = toServer.mutable_calibrate_dmd();
        dmd->set_image(std::string(frame.constData(), frame.size()));

        QByteArray data(static_cast<int>(toServer.ByteSizeLong()), '\0');
        if (!toServer.SerializeToArray(data.data(), data.size())) {
            errorMessage = "Failed to serialize DMD circle request.";
            doLogMessage(errorMessage);
            return false;
        }

        networkService_->sendMessage(data);

        doLogMessage("DMD 5mm circle frame sent to server.");
        return true;
    } catch (const std::exception& e) {
        errorMessage = QString("DMD request failed: %1").arg(e.what());
        doLogMessage(errorMessage);
        return false;
    }
}