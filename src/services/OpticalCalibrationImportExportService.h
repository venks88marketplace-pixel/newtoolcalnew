#pragma once

#include <QObject>
#include <QTimer>
#include <memory>
#include <QUrl>
#include "../interfaces/INetworkService.h"

class OpticalCalibrationImportExportService : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool busy READ busy NOTIFY busyChanged)

public:
    explicit OpticalCalibrationImportExportService(
        std::shared_ptr<INetworkService> network,
        QObject* parent = nullptr);

    bool busy() const { return m_busy; }

    // Starts the asynchronous request and saves its response to filePath.
    Q_INVOKABLE void exportToFile(const QUrl& fileUrl);

    Q_INVOKABLE void importFromFile(const QUrl& fileUrl);

signals:
    void busyChanged();
    void exportSucceeded(const QString& filePath);
    void exportFailed(const QString& reason);

    void importRequestSent();
    void importFailed(const QString& reason);

private slots:
    void onNetworkMessageReceived(const QByteArray& data);
    void onTimeout();

private:
    void setBusy(bool busy);

    std::shared_ptr<INetworkService> m_network;
    QString m_filePath;
    QTimer m_timeout;
    bool m_busy = false;
};