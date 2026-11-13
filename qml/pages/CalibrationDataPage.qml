import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs

Page {
    id: root

    property url pendingImportFile: ""
    property string statusMessage: ""
    property color statusColor: "#444444"

    background: Rectangle {
        color: "#ffffff"
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 40
        spacing: 0

        Label {
            text: "Optical Calibration Data Import/Export"
            font.pixelSize: 20
            font.bold: true
            color: "#222222"
            Layout.alignment: Qt.AlignHCenter
            Layout.topMargin: 10
            Layout.bottomMargin: 24
        }

        Label {
            text: "Import or export optical calibration data."
            color: "#555555"
            Layout.alignment: Qt.AlignHCenter
            Layout.bottomMargin: 70
        }

        GridLayout {
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVStretch
            Layout.fillWidth: true
            Layout.fillHeight: true
            columnSpacing: Math.max(20, root.width * 0.1)
            columns: 2

            NavButton {
                text: "Export Calibration Data"
                enabled: !appContext.opticalCalibrationImportExportService.busy
                Layout.fillWidth: true

                onClicked: exportFileDialog.open()
            }

            NavButton {
                text: "Import Calibration Data"
                enabled: !appContext.opticalCalibrationImportExportService.busy
                Layout.fillWidth: true

                onClicked: importFileDialog.open()
            }
        }

        Label {
            text: root.statusMessage
            color: root.statusColor
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
            Layout.topMargin: 20
            horizontalAlignment: Text.AlignHCenter
            visible: root.statusMessage.length > 0
        }

        Item {
            Layout.fillHeight: true
        }
    }

    FileDialog {
        id: exportFileDialog
        title: "Export optical calibration"
        fileMode: FileDialog.SaveFile
        nameFilters: ["JSON files (*.json)"]
        defaultSuffix: "json"
        currentFile: "optical_calibration.json"

        onAccepted: {
            root.statusMessage = "Exporting calibration data..."
            root.statusColor = "#444444"
            appContext.opticalCalibrationImportExportService.exportToFile(selectedFile)
        }
    }

    FileDialog {
        id: importFileDialog
        title: "Select optical calibration JSON"
        fileMode: FileDialog.OpenFile
        nameFilters: ["JSON files (*.json)"]

        onAccepted: {
            root.pendingImportFile = selectedFile
            importConfirmation.open()
        }
    }

    Dialog {
        id: importConfirmation
        title: "Confirm calibration import"
        modal: true
        anchors.centerIn: Overlay.overlay
        standardButtons: Dialog.Yes | Dialog.Cancel

        contentItem: ColumnLayout {
            spacing: 12
            implicitWidth: 420

            Label {
                text: "Are you sure you want to send this calibration JSON to the head?"
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }

            Label {
                text: root.pendingImportFile.toString()
                wrapMode: Text.WrapAnywhere
                color: "#555555"
                Layout.fillWidth: true
            }

            Label {
                text: "The current server may replace existing calibration data. Continue only if that is intended."
                wrapMode: Text.WordWrap
                color: "#8a5a00"
                Layout.fillWidth: true
            }
        }

        onAccepted: {
            root.statusMessage = "Sending calibration import request..."
            root.statusColor = "#444444"
            appContext.opticalCalibrationImportExportService.importFromFile(
                root.pendingImportFile)
        }
    }

    Connections {
        target: appContext.opticalCalibrationImportExportService

        function onExportSucceeded(filePath) {
            root.statusMessage = "Calibration JSON exported to: " + filePath
            root.statusColor = "#207245"
        }

        function onExportFailed(reason) {
            root.statusMessage = "Export failed: " + reason
            root.statusColor = "#b3261e"
        }

        function onImportRequestSent() {
            root.statusMessage = "Import request sent."
            root.statusColor = "#8a5a00"
        }

        function onImportFailed(reason) {
            root.statusMessage = "Import failed: " + reason
            root.statusColor = "#b3261e"
        }
    }
}