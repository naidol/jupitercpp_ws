// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter animated face — reacts to voice and face recognition state.
// States: 0=LISTENING  1=PROCESSING  2=SPEAKING
// Modes:  0=INTERACTION  1=NAVIGATING

import QtQuick 2.15
import QtQuick.Window 2.15

Window {
    id: root
    color: "#0A0A0F"
    title: "Jupiter"

    // ── Top status bar ─────────────────────────────────────────────────────────
    Rectangle {
        id: topBar
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 28
        color: "#0D0D1A"

        Row {
            anchors.left: parent.left
            anchors.leftMargin: root.width * 0.022
            anchors.verticalCenter: parent.verticalCenter
            spacing: 6

            Rectangle {
                width: 8; height: 8; radius: 4
                anchors.verticalCenter: parent.verticalCenter
                color: bridge.robotMode === 1 ? "#00FF7F" : "#00E5FF"
            }

            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: bridge.robotMode === 1
                      ? ("NAVIGATING" + (bridge.linearVel > 0.05
                         ? "   " + bridge.linearVel.toFixed(2) + " m/s" : ""))
                      : "INTERACTION"
                font.pixelSize: 11
                font.letterSpacing: 0.8
                color: bridge.robotMode === 1 ? "#00FF7F" : "#00E5FF"
            }
        }

        Row {
            anchors.right: parent.right
            anchors.rightMargin: root.width * 0.022
            anchors.verticalCenter: parent.verticalCenter
            spacing: 5

            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: "WiFi"
                font.pixelSize: 10
                color: "#555555"
            }

            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: bridge.wifiSignal < 0 ? "—" : bridge.wifiSignal + "%"
                font.pixelSize: 11
                font.bold: true
                color: bridge.wifiSignal < 0  ? "#FF4444" :
                       bridge.wifiSignal < 30 ? "#FF4444" :
                       bridge.wifiSignal < 60 ? "#FFA500" : "#00FF7F"
            }
        }
    }

    // ── JUPITER title ─────────────────────────────────────────────────────────
    Text {
        id: titleText
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: topBar.bottom
        anchors.topMargin: root.height * 0.032
        text: "J U P I T E R"
        font.pixelSize: root.height * 0.068
        font.bold: true
        font.letterSpacing: root.height * 0.011
        color: "#00E5FF"
        style: Text.Outline
        styleColor: "#004D7A"

        SequentialAnimation on opacity {
            running: bridge.state === 1
            loops: Animation.Infinite
            NumberAnimation { to: 0.5; duration: 600 }
            NumberAnimation { to: 1.0; duration: 600 }
        }
    }

    // ── Face container ────────────────────────────────────────────────────────
    Item {
        id: faceContainer
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: titleText.bottom
        anchors.topMargin: root.height * 0.022
        width: root.width * 0.72
        height: root.height * 0.50

        // ── Left eye ──────────────────────────────────────────────────────────
        Item {
            id: leftEyeItem
            x: parent.width * 0.08
            y: parent.height * 0.05
            width: parent.width * 0.32
            height: parent.height * 0.45

            transform: Scale {
                id: leftBlinkScale
                origin.x: leftEyeItem.width / 2
                origin.y: leftEyeItem.height / 2
                yScale: 1.0
            }

            Rectangle {
                anchors.fill: parent
                radius: width * 0.45
                color: bridge.state === 1 ? "#CCCCCC" : "#FFFFFF"
                Behavior on color { ColorAnimation { duration: 200 } }

                Rectangle {
                    id: leftPupil
                    anchors.centerIn: parent
                    width: parent.width * 0.44
                    height: parent.height * 0.44
                    radius: width / 2
                    color: "#111111"

                    Rectangle {
                        x: parent.width * 0.55
                        y: parent.height * 0.12
                        width: parent.width * 0.28
                        height: parent.height * 0.28
                        radius: width / 2
                        color: "#FFFFFF"
                    }
                }

                Rectangle {
                    visible: bridge.state === 1
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    height: parent.height * 0.45
                    radius: 0
                    color: bridge.state === 1 ? "#CCCCCC" : "transparent"
                    Behavior on color { ColorAnimation { duration: 300 } }
                }
            }
        }

        // ── Right eye ─────────────────────────────────────────────────────────
        Item {
            id: rightEyeItem
            x: parent.width * 0.60
            y: parent.height * 0.05
            width: parent.width * 0.32
            height: parent.height * 0.45

            transform: Scale {
                id: rightBlinkScale
                origin.x: rightEyeItem.width / 2
                origin.y: rightEyeItem.height / 2
                yScale: 1.0
            }

            Rectangle {
                anchors.fill: parent
                radius: width * 0.45
                color: bridge.state === 1 ? "#CCCCCC" : "#FFFFFF"
                Behavior on color { ColorAnimation { duration: 200 } }

                Rectangle {
                    anchors.centerIn: parent
                    width: parent.width * 0.44
                    height: parent.height * 0.44
                    radius: width / 2
                    color: "#111111"

                    Rectangle {
                        x: parent.width * 0.55
                        y: parent.height * 0.12
                        width: parent.width * 0.28
                        height: parent.height * 0.28
                        radius: width / 2
                        color: "#FFFFFF"
                    }
                }

                Rectangle {
                    visible: bridge.state === 1
                    anchors.bottom: parent.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    height: parent.height * 0.45
                    radius: 0
                    color: bridge.state === 1 ? "#CCCCCC" : "transparent"
                    Behavior on color { ColorAnimation { duration: 300 } }
                }
            }
        }

        // ── Mouth (Canvas) ────────────────────────────────────────────────────
        Canvas {
            id: mouthCanvas
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: parent.height * 0.02
            width: parent.width * 0.55
            height: parent.height * 0.28

            property real openAmount: 0.0

            onOpenAmountChanged: requestPaint()
            onPaint: {
                var ctx = getContext("2d");
                ctx.clearRect(0, 0, width, height);
                var cx = width / 2;
                var state = bridge.state;

                if (bridge.sleeping) {
                    // Flat closed line — neutral sleeping expression
                    ctx.beginPath();
                    ctx.moveTo(cx - width * 0.16, height * 0.50);
                    ctx.lineTo(cx + width * 0.16, height * 0.50);
                    ctx.strokeStyle = "#333355";
                    ctx.lineWidth = height * 0.07;
                    ctx.lineCap = "round";
                    ctx.stroke();

                } else if (state === 2) {
                    var ow = width * 0.42;
                    var oh = height * (0.28 + openAmount * 0.45);
                    var oy = height * 0.38;
                    ctx.beginPath();
                    ctx.moveTo(cx - ow, oy);
                    ctx.quadraticCurveTo(cx, oy + oh * 0.5,  cx + ow, oy);
                    ctx.quadraticCurveTo(cx, oy - oh * 0.15, cx - ow, oy);
                    ctx.fillStyle = "#CC1A1A";
                    ctx.fill();
                    ctx.save();
                    ctx.clip();
                    ctx.fillStyle = "#F0F0F0";
                    ctx.fillRect(cx - ow * 0.7, oy - oh * 0.05, ow * 1.4, oh * 0.28);
                    ctx.restore();
                    ctx.beginPath();
                    ctx.moveTo(cx - ow, oy);
                    ctx.quadraticCurveTo(cx, oy + oh * 0.5, cx + ow, oy);
                    ctx.quadraticCurveTo(cx, oy - oh * 0.15, cx - ow, oy);
                    ctx.strokeStyle = "#FFFFFF";
                    ctx.lineWidth = height * 0.06;
                    ctx.stroke();

                } else if (state === 0) {
                    ctx.beginPath();
                    ctx.moveTo(cx - width * 0.35, height * 0.32);
                    ctx.quadraticCurveTo(cx, height * 0.82, cx + width * 0.35, height * 0.32);
                    ctx.strokeStyle = "#FFFFFF";
                    ctx.lineWidth = height * 0.09;
                    ctx.lineCap = "round";
                    ctx.stroke();

                } else {
                    ctx.beginPath();
                    ctx.moveTo(cx - width * 0.28, height * 0.5);
                    ctx.lineTo(cx + width * 0.28, height * 0.5);
                    ctx.strokeStyle = "#888888";
                    ctx.lineWidth = height * 0.07;
                    ctx.lineCap = "round";
                    ctx.stroke();
                }
            }

            SequentialAnimation on openAmount {
                running: bridge.state === 2
                loops: Animation.Infinite
                NumberAnimation { to: 1.0; duration: 180; easing.type: Easing.SineCurve }
                NumberAnimation { to: 0.2; duration: 180; easing.type: Easing.SineCurve }
            }
            NumberAnimation on openAmount {
                running: bridge.state !== 2
                to: 0.0
                duration: 200
            }

            Connections {
                target: bridge
                function onStateChanged()    { mouthCanvas.requestPaint(); }
                function onSleepingChanged() { mouthCanvas.requestPaint(); }
            }
        }
    }

    // ── Eye blink animations ──────────────────────────────────────────────────
    SequentialAnimation {
        id: blinkAnim
        running: bridge.state !== 1 && !bridge.sleeping
        loops: Animation.Infinite

        PauseAnimation { duration: 2800 }
        NumberAnimation {
            targets: [leftBlinkScale, rightBlinkScale]
            property: "yScale"; to: 0.08; duration: 70
        }
        NumberAnimation {
            targets: [leftBlinkScale, rightBlinkScale]
            property: "yScale"; to: 1.0; duration: 70
        }
        PauseAnimation { duration: 120 }
        NumberAnimation {
            targets: [leftBlinkScale, rightBlinkScale]
            property: "yScale"; to: 0.08; duration: 70
        }
        NumberAnimation {
            targets: [leftBlinkScale, rightBlinkScale]
            property: "yScale"; to: 1.0; duration: 70
        }
    }

    // Sleep — slowly close eyes to a thin slit and hold
    NumberAnimation {
        targets: [leftBlinkScale, rightBlinkScale]
        property: "yScale"
        running: bridge.sleeping
        to: 0.06; duration: 800
        easing.type: Easing.InOutQuad
    }
    // Wake — open eyes
    NumberAnimation {
        targets: [leftBlinkScale, rightBlinkScale]
        property: "yScale"
        running: !bridge.sleeping && bridge.state !== 1
        to: 1.0; duration: 400
        easing.type: Easing.OutQuad
    }
    NumberAnimation {
        targets: [leftBlinkScale, rightBlinkScale]
        property: "yScale"
        running: bridge.state === 1 && !bridge.sleeping
        to: 0.45; duration: 300
    }

    // ── User greeting ─────────────────────────────────────────────────────────
    Text {
        id: userText
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: faceContainer.bottom
        anchors.topMargin: root.height * 0.016
        text: bridge.currentUser === "Unknown" ? "" : "Hello, " + bridge.currentUser
        font.pixelSize: root.height * 0.046
        font.bold: true
        color: "#FFD700"
        visible: bridge.currentUser !== "Unknown"

        Behavior on text {
            SequentialAnimation {
                NumberAnimation { target: userText; property: "opacity"; to: 0; duration: 200 }
                NumberAnimation { target: userText; property: "opacity"; to: 1; duration: 300 }
            }
        }
    }

    // ── Processing dots ───────────────────────────────────────────────────────
    Row {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: faceContainer.bottom
        anchors.topMargin: root.height * 0.022
        spacing: root.width * 0.025
        visible: bridge.state === 1

        Repeater {
            model: 3
            Rectangle {
                width: root.width * 0.022
                height: width
                radius: width / 2
                color: "#00E5FF"

                SequentialAnimation on opacity {
                    running: bridge.state === 1
                    loops: Animation.Infinite
                    PauseAnimation { duration: index * 220 }
                    NumberAnimation { to: 1.0; duration: 300 }
                    NumberAnimation { to: 0.15; duration: 300 }
                    PauseAnimation { duration: (2 - index) * 220 }
                }
            }
        }
    }

    // ── Bottom status bar ─────────────────────────────────────────────────────
    Rectangle {
        id: statusBar
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: 72
        color: "#0D0D1A"

        // Left — voice state
        Row {
            anchors.left: parent.left
            anchors.leftMargin: root.width * 0.025
            anchors.verticalCenter: parent.verticalCenter
            spacing: 7

            Rectangle {
                width: 10; height: 10; radius: 5
                anchors.verticalCenter: parent.verticalCenter
                color: bridge.sleeping    ? "#444466"   :
                       bridge.state === 0 ? "#00FF7F"   :
                       bridge.state === 1 ? "#FFA500"   : "#00AAFF"

                SequentialAnimation on opacity {
                    running: bridge.state === 1 && !bridge.sleeping
                    loops: Animation.Infinite
                    NumberAnimation { to: 0.2; duration: 400 }
                    NumberAnimation { to: 1.0; duration: 400 }
                }
            }

            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: bridge.sleeping    ? "SLEEPING"   :
                      bridge.state === 0 ? "LISTENING"  :
                      bridge.state === 1 ? "PROCESSING" : "SPEAKING"
                font.pixelSize: 13
                font.letterSpacing: 0.8
                color: bridge.sleeping    ? "#444466"   :
                       bridge.state === 0 ? "#00FF7F"   :
                       bridge.state === 1 ? "#FFA500"   : "#00AAFF"
            }
        }

        // Center — sensor health dots
        Row {
            anchors.centerIn: parent
            spacing: root.width * 0.038

            Repeater {
                model: [
                    { label: "CAM",   online: bridge.cameraOnline },
                    { label: "LIDAR", online: bridge.lidarOnline  },
                    { label: "VOICE", online: bridge.voiceOnline  }
                ]

                Column {
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 4

                    Rectangle {
                        width: 10; height: 10; radius: 5
                        anchors.horizontalCenter: parent.horizontalCenter
                        color: modelData.online ? "#00FF7F" : "#FF4444"

                        SequentialAnimation on opacity {
                            running: !modelData.online
                            loops: Animation.Infinite
                            NumberAnimation { to: 0.15; duration: 500 }
                            NumberAnimation { to: 1.0;  duration: 500 }
                        }
                    }

                    Text {
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: modelData.label
                        font.pixelSize: 9
                        color: modelData.online ? "#00FF7F" : "#FF4444"
                    }
                }
            }
        }

        // Right — CPU temp, GPU temp, battery
        Row {
            anchors.right: parent.right
            anchors.rightMargin: root.width * 0.025
            anchors.verticalCenter: parent.verticalCenter
            spacing: root.width * 0.024

            // CPU temp
            Column {
                anchors.verticalCenter: parent.verticalCenter
                spacing: 2

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: bridge.cpuTemp > 0 ? bridge.cpuTemp.toFixed(0) + "°" : "—"
                    font.pixelSize: 13
                    font.bold: true
                    color: bridge.cpuTemp > 85 ? "#FF4444" :
                           bridge.cpuTemp > 70 ? "#FFA500" : "#888888"
                }
                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: "CPU"
                    font.pixelSize: 9
                    color: "#444444"
                }
            }

            // GPU temp
            Column {
                anchors.verticalCenter: parent.verticalCenter
                spacing: 2

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: bridge.gpuTemp > 0 ? bridge.gpuTemp.toFixed(0) + "°" : "—"
                    font.pixelSize: 13
                    font.bold: true
                    color: bridge.gpuTemp > 85 ? "#FF4444" :
                           bridge.gpuTemp > 70 ? "#FFA500" : "#888888"
                }
                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: "GPU"
                    font.pixelSize: 9
                    color: "#444444"
                }
            }

            // Battery bar + % + voltage
            Column {
                anchors.verticalCenter: parent.verticalCenter
                spacing: 3
                visible: bridge.batteryVoltage > 0 || bridge.batteryPct > 0

                Row {
                    anchors.horizontalCenter: parent.horizontalCenter
                    spacing: 5

                    Rectangle {
                        width: 38; height: 11; radius: 2
                        anchors.verticalCenter: parent.verticalCenter
                        color: "transparent"
                        border.color: bridge.batteryPct < 20 ? "#FF4444" :
                                      bridge.batteryPct < 50 ? "#FFA500" : "#00FF7F"
                        border.width: 1

                        Rectangle {
                            x: 1; y: 1
                            width: Math.max(2, (parent.width - 2) * bridge.batteryPct / 100)
                            height: parent.height - 2
                            radius: 1
                            color: bridge.batteryPct < 20 ? "#FF4444" :
                                   bridge.batteryPct < 50 ? "#FFA500" : "#00FF7F"
                        }
                    }

                    Text {
                        anchors.verticalCenter: parent.verticalCenter
                        text: bridge.batteryPct > 0 ? bridge.batteryPct.toFixed(0) + "%" : "—"
                        font.pixelSize: 12
                        font.bold: true
                        color: bridge.batteryPct < 20 ? "#FF4444" :
                               bridge.batteryPct < 50 ? "#FFA500" : "#00FF7F"
                    }
                }

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: bridge.batteryVoltage > 0 ? bridge.batteryVoltage.toFixed(1) + "V" : ""
                    font.pixelSize: 9
                    color: "#444444"
                }
            }
        }
    }
}
