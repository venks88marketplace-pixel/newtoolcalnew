import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    anchors.fill: parent

    Text {
        anchors.centerIn: parent
        text: "Page is ready to be deleted"
        font.pixelSize: 24
        font.bold: true
        color: "black"
    }
}