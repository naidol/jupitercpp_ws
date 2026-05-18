// Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
// SPDX-License-Identifier: Apache-2.0
//
// Jupiter animated face — reacts to voice and face recognition state.
// States: 0=LISTENING  1=PROCESSING  2=SPEAKING

import QtQuick 2.15
import QtQuick.Window 2.15

Window {
    id: root
    color: "#0A0A0F"
    title: "Jupiter"


    // ── Auto-return to LISTENING after TTS finishes ───────────────────────────
    Timer {
        id: speakingTimer
        interval: bridge.speakingMs
        running: bridge.state === 2
        repeat: false
        onTriggered: bridge.returnToListening()
    }

    // ── JUPITER title ─────────────────────────────────────────────────────────
    Text {
        id: titleText
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: root.height * 0.04
        text: "J U P I T E R"
        font.pixelSize: root.height * 0.075
        font.bold: true
        font.letterSpacing: root.height * 0.012
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
        anchors.topMargin: root.height * 0.03
        width: root.width * 0.72
        height: root.height * 0.58

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

            // White of eye
            Rectangle {
                anchors.fill: parent
                radius: width * 0.45
                color: bridge.state === 1 ? "#CCCCCC" : "#FFFFFF"
                Behavior on color { ColorAnimation { duration: 200 } }

                // Pupil
                Rectangle {
                    id: leftPupil
                    anchors.centerIn: parent
                    width: parent.width * 0.44
                    height: parent.height * 0.44
                    radius: width / 2
                    color: "#111111"

                    // Highlight
                    Rectangle {
                        x: parent.width * 0.55
                        y: parent.height * 0.12
                        width: parent.width * 0.28
                        height: parent.height * 0.28
                        radius: width / 2
                        color: "#FFFFFF"
                    }
                }

                // Thinking squint overlay (PROCESSING state)
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

                if (state === 2) {
                    // SPEAKING — open mouth, animates with openAmount
                    var ow = width * 0.42;
                    var oh = height * (0.28 + openAmount * 0.45);
                    var oy = height * 0.38;
                    // Outer lip shape
                    ctx.beginPath();
                    ctx.moveTo(cx - ow, oy);
                    ctx.quadraticCurveTo(cx, oy + oh * 0.5,  cx + ow, oy);
                    ctx.quadraticCurveTo(cx, oy - oh * 0.15, cx - ow, oy);
                    ctx.fillStyle = "#CC1A1A";
                    ctx.fill();
                    // Teeth
                    ctx.save();
                    ctx.clip();
                    ctx.fillStyle = "#F0F0F0";
                    ctx.fillRect(cx - ow * 0.7, oy - oh * 0.05, ow * 1.4, oh * 0.28);
                    ctx.restore();
                    // Lip outline
                    ctx.beginPath();
                    ctx.moveTo(cx - ow, oy);
                    ctx.quadraticCurveTo(cx, oy + oh * 0.5, cx + ow, oy);
                    ctx.quadraticCurveTo(cx, oy - oh * 0.15, cx - ow, oy);
                    ctx.strokeStyle = "#FFFFFF";
                    ctx.lineWidth = height * 0.06;
                    ctx.stroke();

                } else if (state === 0) {
                    // LISTENING — gentle smile
                    ctx.beginPath();
                    ctx.moveTo(cx - width * 0.35, height * 0.32);
                    ctx.quadraticCurveTo(cx, height * 0.82, cx + width * 0.35, height * 0.32);
                    ctx.strokeStyle = "#FFFFFF";
                    ctx.lineWidth = height * 0.09;
                    ctx.lineCap = "round";
                    ctx.stroke();

                } else {
                    // PROCESSING — flat line with thinking dots
                    ctx.beginPath();
                    ctx.moveTo(cx - width * 0.28, height * 0.5);
                    ctx.lineTo(cx + width * 0.28, height * 0.5);
                    ctx.strokeStyle = "#888888";
                    ctx.lineWidth = height * 0.07;
                    ctx.lineCap = "round";
                    ctx.stroke();
                }
            }

            // Speaking mouth oscillation
            SequentialAnimation on openAmount {
                running: bridge.state === 2
                loops: Animation.Infinite
                NumberAnimation { to: 1.0; duration: 180; easing.type: Easing.SineCurve }
                NumberAnimation { to: 0.2; duration: 180; easing.type: Easing.SineCurve }
            }
            // Reset mouth when not speaking
            NumberAnimation on openAmount {
                running: bridge.state !== 2
                to: 0.0
                duration: 200
            }

            // Repaint when state changes
            Connections {
                target: bridge
                function onStateChanged() { mouthCanvas.requestPaint(); }
            }
        }
    }

    // ── Eye blink animations ──────────────────────────────────────────────────
    SequentialAnimation {
        id: blinkAnim
        running: bridge.state !== 1
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

    // Squint when processing
    NumberAnimation {
        targets: [leftBlinkScale, rightBlinkScale]
        property: "yScale"
        running: bridge.state === 1
        to: 0.45; duration: 300
    }
    NumberAnimation {
        targets: [leftBlinkScale, rightBlinkScale]
        property: "yScale"
        running: bridge.state !== 1
        to: 1.0; duration: 200
    }

    // ── User greeting ─────────────────────────────────────────────────────────
    Text {
        id: userText
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: faceContainer.bottom
        anchors.topMargin: root.height * 0.025
        text: bridge.currentUser === "Unknown" ? "" : "Hello, " + bridge.currentUser
        font.pixelSize: root.height * 0.052
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
        anchors.topMargin: root.height * 0.03
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

    // ── Status bar ────────────────────────────────────────────────────────────
    Rectangle {
        id: statusBar
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: root.height * 0.075
        color: "#111118"

        Row {
            anchors.centerIn: parent
            spacing: root.width * 0.04

            // State indicator
            Rectangle {
                width: root.width * 0.016
                height: width
                radius: width / 2
                anchors.verticalCenter: parent.verticalCenter
                color: bridge.state === 0 ? "#00FF7F" :
                       bridge.state === 1 ? "#FFA500" : "#00AAFF"

                SequentialAnimation on opacity {
                    running: bridge.state === 1
                    loops: Animation.Infinite
                    NumberAnimation { to: 0.2; duration: 400 }
                    NumberAnimation { to: 1.0; duration: 400 }
                }
            }

            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: bridge.state === 0 ? "LISTENING" :
                      bridge.state === 1 ? "PROCESSING" : "SPEAKING"
                font.pixelSize: statusBar.height * 0.38
                font.letterSpacing: 1
                color: bridge.state === 0 ? "#00FF7F" :
                       bridge.state === 1 ? "#FFA500" : "#00AAFF"
            }

            // Battery (only shown when data is available)
            Text {
                visible: bridge.batteryVoltage > 0
                anchors.verticalCenter: parent.verticalCenter
                text: "⬡ " + bridge.batteryVoltage.toFixed(1) + "V"
                font.pixelSize: statusBar.height * 0.38
                color: bridge.batteryVoltage < 14.4 ? "#FF4444" : "#888888"
            }
        }
    }
}
