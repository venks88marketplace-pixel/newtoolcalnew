#include "OpticalCalibrationImportExportService.h"

#include <QDebug>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QFile>
#include <google/protobuf/util/json_util.h>
#include "neo-calibration-server-api/calibration_server_api.pb.h"

namespace
{
QJsonObject matrixToJson(const CalibrationApi::Matrix& matrix)
{
    QJsonObject result;
    result.insert(QStringLiteral("rows"), static_cast<int>(matrix.rows()));
    result.insert(QStringLiteral("cols"), static_cast<int>(matrix.cols()));

    QJsonArray data;
    for (int i = 0; i < matrix.data_size(); ++i) {
        data.append(matrix.data(i));
    }
    result.insert(QStringLiteral("data"), data);

    return result;
}

QJsonObject nfovToJson(const CalibrationApi::NfovCalibration& calibration)
{
    QJsonObject result;
    result.insert(QStringLiteral("date_string"),
                  QString::fromStdString(calibration.date_string()));

    result.insert(QStringLiteral("left_distortion_matrix"),
                  matrixToJson(calibration.left_distortion_matrix()));
    result.insert(QStringLiteral("left_intrinsic_matrix"),
                  matrixToJson(calibration.left_intrinsic_matrix()));
    result.insert(QStringLiteral("left_projection_matrix"),
                  matrixToJson(calibration.left_projection_matrix()));
    result.insert(QStringLiteral("right_distortion_matrix"),
                  matrixToJson(calibration.right_distortion_matrix()));
    result.insert(QStringLiteral("right_intrinsic_matrix"),
                  matrixToJson(calibration.right_intrinsic_matrix()));
    result.insert(QStringLiteral("right_projection_matrix"),
                  matrixToJson(calibration.right_projection_matrix()));
    result.insert(QStringLiteral("fundamental_matrix"),
                  matrixToJson(calibration.fundamental_matrix()));
    result.insert(QStringLiteral("essential_matrix"),
                  matrixToJson(calibration.essential_matrix()));
    result.insert(QStringLiteral("stereo_rotation_matrix"),
                  matrixToJson(calibration.stereo_rotation_matrix()));
    result.insert(QStringLiteral("stereo_translation_matrix"),
                  matrixToJson(calibration.stereo_translation_matrix()));
    result.insert(QStringLiteral("rectify_left_matrix"),
                  matrixToJson(calibration.rectify_left_matrix()));
    result.insert(QStringLiteral("rectify_right_matrix"),
                  matrixToJson(calibration.rectify_right_matrix()));
    result.insert(QStringLiteral("rectified_projection_left_matrix"),
                  matrixToJson(calibration.rectified_projection_left_matrix()));
    result.insert(QStringLiteral("rectified_projection_right_matrix"),
                  matrixToJson(calibration.rectified_projection_right_matrix()));
    result.insert(QStringLiteral("disparity_to_z_matrix"),
                  matrixToJson(calibration.disparity_to_z_matrix()));
    result.insert(QStringLiteral("transform_to_instrument_axis_matrix"),
                  matrixToJson(calibration.transform_to_instrument_axis_matrix()));

    result.insert(QStringLiteral("left_roi_x"),
                  static_cast<int>(calibration.left_roi_x()));
    result.insert(QStringLiteral("left_roi_y"),
                  static_cast<int>(calibration.left_roi_y()));
    result.insert(QStringLiteral("right_roi_x"),
                  static_cast<int>(calibration.right_roi_x()));
    result.insert(QStringLiteral("right_roi_y"),
                  static_cast<int>(calibration.right_roi_y()));

    return result;
}

QJsonObject wfovToJson(const CalibrationApi::WfovCalibration& calibration)
{
    QJsonObject result;
    result.insert(QStringLiteral("date_string"),
                  QString::fromStdString(calibration.date_string()));
    result.insert(QStringLiteral("lab_center_x"), calibration.lab_center_x());
    result.insert(QStringLiteral("lab_center_y"), calibration.lab_center_y());
    result.insert(QStringLiteral("lab_center_z"), calibration.lab_center_z());
    return result;
}

QJsonObject photometricToJson(
    const CalibrationApi::PhotometricCalibration& calibration)
{
    QJsonObject result;
    result.insert(QStringLiteral("date_string"),
                  QString::fromStdString(calibration.date_string()));
    result.insert(QStringLiteral("exposure"),
                  static_cast<int>(calibration.exposure()));
    result.insert(QStringLiteral("left_gain"), calibration.left_gain());
    result.insert(QStringLiteral("right_gain"), calibration.right_gain());
    return result;
}

QJsonObject irToJson(const CalibrationApi::IrCalibration& calibration)
{
    QJsonObject result;
    result.insert(QStringLiteral("date_string"),
                  QString::fromStdString(calibration.date_string()));
    result.insert(QStringLiteral("ir_top_percent"),
                  static_cast<int>(calibration.ir_top_percent()));
    result.insert(QStringLiteral("ir_bottom_percent"),
                  static_cast<int>(calibration.ir_bottom_percent()));
    return result;
}

QJsonObject dmdToJson(const CalibrationApi::DmdCalibration& calibration)
{
    QJsonObject result;
    result.insert(QStringLiteral("date_string"),
                  QString::fromStdString(calibration.date_string()));

    if (calibration.has_gk()) {
        result.insert(QStringLiteral("gK"),
                      matrixToJson(calibration.gk()));
    }
    if (calibration.has_gd()) {
        result.insert(QStringLiteral("gD"),
                      matrixToJson(calibration.gd()));
    }
    if (calibration.has_gr()) {
        result.insert(QStringLiteral("gR"),
                      matrixToJson(calibration.gr()));
    }
    if (calibration.has_gt()) {
        result.insert(QStringLiteral("gT"),
                      matrixToJson(calibration.gt()));
    }

    return result;
}

QJsonObject opticalCalibrationToJson(
    const CalibrationApi::OpticalCalibration& calibration)
{
    QJsonObject result;

    if (calibration.has_nfov_calibration()) {
        result.insert(QStringLiteral("nfov_calibration"),
                      nfovToJson(calibration.nfov_calibration()));
    }
    if (calibration.has_wfov_calibration()) {
        result.insert(QStringLiteral("wfov_calibration"),
                      wfovToJson(calibration.wfov_calibration()));
    }
    if (calibration.has_photometric_calibration()) {
        result.insert(QStringLiteral("photometric_calibration"),
                      photometricToJson(
                          calibration.photometric_calibration()));
    }
    if (calibration.has_dmd_calibration()) {
        result.insert(QStringLiteral("dmd_calibration"),
                      dmdToJson(calibration.dmd_calibration()));
    }
    if (calibration.has_ir_calibration()) {
        result.insert(QStringLiteral("ir_calibration"),
                      irToJson(calibration.ir_calibration()));
    }

    return result;
}

bool saveJsonFile(const QString& filePath, const QJsonObject& json,
                  QString* error)
{
    QSaveFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        *error = file.errorString();
        return false;
    }

    const QByteArray bytes =
        QJsonDocument(json).toJson(QJsonDocument::Indented);

    if (file.write(bytes) != bytes.size()) {
        *error = file.errorString();
        file.cancelWriting();
        return false;
    }

    if (!file.commit()) {
        *error = file.errorString();
        return false;
    }

    return true;
}
} // namespace

OpticalCalibrationImportExportService::OpticalCalibrationImportExportService(
    std::shared_ptr<INetworkService> network,
    QObject* parent)
    : QObject(parent),
      m_network(std::move(network))
{
    m_timeout.setSingleShot(true);
    m_timeout.setInterval(10000);

    connect(&m_timeout, &QTimer::timeout,
            this, &OpticalCalibrationImportExportService::onTimeout);

    if (m_network) {
        connect(m_network.get(), &INetworkService::messageReceived,
                this, &OpticalCalibrationImportExportService::onNetworkMessageReceived);
    }
}

void OpticalCalibrationImportExportService::setBusy(bool busy)
{
    if (m_busy == busy) {
        return;
    }

    m_busy = busy;
    emit busyChanged();
}

void OpticalCalibrationImportExportService::exportToFile(const QUrl& fileUrl)
{
    if (m_busy) {
        emit exportFailed(QStringLiteral("An optical calibration export is already in progress."));
        return;
    }

    const QString outputPath = fileUrl.toLocalFile().trimmed();
    if (outputPath.isEmpty()) {
        emit exportFailed(QStringLiteral("No local output file was selected."));
        return;
    }

    if (!m_network || !m_network->isConnected()) {
        emit exportFailed(QStringLiteral("Network not connected."));
        return;
    }

    m_filePath = outputPath;

    CalibrationApi::ToServer request;
    request.mutable_get_optical_calibration();

    QByteArray requestBytes(
        static_cast<int>(request.ByteSizeLong()), '\0');

    if (!request.SerializeToArray(requestBytes.data(), requestBytes.size())) {
        m_filePath.clear();
        emit exportFailed(QStringLiteral("Failed to serialize GetOpticalCalibration request."));
        return;
    }

    setBusy(true);
    m_timeout.start();
    m_network->sendMessage(requestBytes);
}

void OpticalCalibrationImportExportService::importFromFile(const QUrl& fileUrl)
{
    if (!m_network || !m_network->isConnected()) {
        emit importFailed(QStringLiteral("Network not connected."));
        return;
    }

    if (!fileUrl.isLocalFile()) {
        emit importFailed(QStringLiteral("Please select a local JSON file."));
        return;
    }

    const QString filePath = fileUrl.toLocalFile();
    QFile file(filePath);

    if (!file.open(QIODevice::ReadOnly)) {
        emit importFailed(
            QStringLiteral("Could not open %1: %2")
                .arg(filePath, file.errorString()));
        return;
    }

    const QByteArray jsonBytes = file.readAll();
    if (file.error() != QFileDevice::NoError) {
        emit importFailed(
            QStringLiteral("Could not read %1: %2")
                .arg(filePath, file.errorString()));
        return;
    }

    CalibrationApi::OpticalCalibration calibration;
    const auto parseStatus =
        google::protobuf::util::JsonStringToMessage(
            jsonBytes.toStdString(), &calibration);

    if (!parseStatus.ok()) {
        emit importFailed(
            QStringLiteral("Invalid optical calibration JSON: %1")
                .arg(QString::fromStdString(parseStatus.ToString())));
        return;
    }

    if (!calibration.has_nfov_calibration()
        && !calibration.has_wfov_calibration()
        && !calibration.has_photometric_calibration()
        && !calibration.has_ir_calibration()
        && !calibration.has_dmd_calibration()) {
        emit importFailed(
            QStringLiteral("The JSON contains no calibration sections."));
        return;
    }

    CalibrationApi::ToServer request;
    request.mutable_set_optical_calibration()
        ->mutable_optical_calibration()
        ->CopyFrom(calibration);

    QByteArray requestBytes(
        static_cast<int>(request.ByteSizeLong()), '\0');

    if (!request.SerializeToArray(requestBytes.data(), requestBytes.size())) {
        emit importFailed(
            QStringLiteral("Failed to serialize SetOpticalCalibration request."));
        return;
    }

    m_network->sendMessage(requestBytes);
    emit importRequestSent();
}

void OpticalCalibrationImportExportService::onNetworkMessageReceived(
    const QByteArray& data)
{
    if (!m_busy) {
        return;
    }

    CalibrationApi::FromServer response;
    if (!response.ParseFromArray(data.constData(), data.size())) {
        return;
    }

    if (!response.has_optical_calibration()) {
        return;
    }

    m_timeout.stop();

    const QJsonObject json =
        opticalCalibrationToJson(response.optical_calibration());

    QString error;
    if (!saveJsonFile(m_filePath, json, &error)) {
        const QString failedPath = m_filePath;
        m_filePath.clear();
        setBusy(false);
        emit exportFailed(
            QStringLiteral("Could not save %1: %2").arg(failedPath, error));
        return;
    }

    const QString savedPath = m_filePath;
    m_filePath.clear();
    setBusy(false);
    emit exportSucceeded(savedPath);
}

void OpticalCalibrationImportExportService::onTimeout()
{
    const QString failedPath = m_filePath;
    m_filePath.clear();
    setBusy(false);

    emit exportFailed(
        QStringLiteral("Timed out waiting for optical calibration data for %1.")
            .arg(failedPath));
}