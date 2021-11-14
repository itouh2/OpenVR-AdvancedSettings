import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.3
import ovras.advsettings 1.0
import "common"


MyStackViewPage {
    headerText: "Quick Origin"

    content: ColumnLayout {
        spacing: 18

        Timer {
            id: quickOriginUpdateTimer
            repeat: true
            interval: 100
            onTriggered: {
            }
        }

        onVisibleChanged: {
            if (visible) {
            } else {
            }
        }

    }

}
