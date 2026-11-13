import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Page {
    id: root

    // Signal emitted when any screen navigation button is clicked
    signal navigateRequested(string targetScreen)

    background: Rectangle {
        color: "#ffffff"
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 40
        Layout.fillHeight: true
        Layout.fillWidth: true
        spacing: 0

        Label {
            text: "Home"
            font.pixelSize: 20
            font.bold: true
            color: "#222222"
            Layout.alignment: Qt.AlignHCenter
            Layout.topMargin: 10
            Layout.bottomMargin: 70
        }

        GridLayout {
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVStretch
            Layout.fillWidth: true
            Layout.fillHeight: true 
            columnSpacing: Math.max(20, root.width * 0.1)
            rowSpacing: 80
            columns: 2

            NavButton {
                text: "UV Assembly Calibration"
                Layout.fillWidth: true 
                onClicked: { 
                    nav.push("qrc:/qt/qml/App/qml/pages/UVLaunchAssemblyPage.qml")                 
                }
            }

            NavButton {
                text: "WFOV Assembly Calibration"
                Layout.fillWidth: true 
                onClicked: { 
                    nav.push("qrc:/qt/qml/App/qml/pages/WfovAssemblyPage.qml")                 
                }                
            }

            NavButton {
                text: "NFOV Assembly Calibration"
                Layout.fillWidth: true 
                onClicked: { 
                    nav.push("qrc:/qt/qml/App/qml/pages/NfovAssemblyPage.qml")                 
                }                
            }

            NavButton {
                text: "Head Assembly Calibration"
                Layout.fillWidth: true 
                onClicked: { 
                    nav.push("qrc:/qt/qml/App/qml/pages/HeadAssemblySelectionPage.qml")                 
                }                
            }

            NavButton {
                text: "Import / Export Calibration\nData"
                Layout.fillWidth: true
                onClicked: {
                    nav.push("qrc:/qt/qml/App/qml/pages/CalibrationDataPage.qml")
                }
            }

            NavButton {
                 Layout.alignment: Qt.AlignHCenter
                 text: "Main Tabs Page"
                 Layout.columnSpan: 2
                 onClicked: {
                    nav.push("qrc:/qt/qml/App/qml/pages/MainTabsPage.qml")
                }
            }
        }

        Item { Layout.fillHeight: true } // Flexible spacer below grid
    }
}
