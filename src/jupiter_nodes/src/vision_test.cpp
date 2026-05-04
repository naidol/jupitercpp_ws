#include <iostream>
#include <opencv2/opencv.hpp>
#include <vpi/VPI.h>
#include <vpi/Image.h>
#include <vpi/Stream.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/AprilTags.h>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) return -1;

    std::cout << "--- Jupiter AprilTag Sight (VPI 4.0 / PVA) ---" << std::endl;

    VPIStream stream;
    vpiStreamCreate(0, &stream);

    VPIAprilTagDecodeParams decodeParams;
    vpiInitAprilTagDecodeParams(&decodeParams);
    decodeParams.family = VPI_APRILTAG_36H11;

    VPIPayload payload;
    vpiCreateAprilTagDetector(VPI_BACKEND_PVA, 640, 480, &decodeParams, &payload);

    VPIArray detections;
    vpiArrayCreate(10, VPI_ARRAY_TYPE_APRILTAG_DETECTION, VPI_BACKEND_CPU, &detections);

    cv::Mat frame, grayFrame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Handle both RGB and IR/Grayscale feeds dynamically
        if (frame.channels() == 3) {
            cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
        } else if (frame.channels() == 1) {
            grayFrame = frame; // It's already grayscale (Orbbec IR Feed)
        } else {
            std::cerr << "Jupiter Error: Unrecognized camera format." << std::endl;
            break;
        }

        VPIImage vpiFrame = nullptr;
        vpiImageCreateWrapperOpenCVMat(grayFrame, VPI_IMAGE_FORMAT_U8, 0, &vpiFrame);

        vpiSubmitAprilTagDetector(stream, VPI_BACKEND_PVA, payload, 10, vpiFrame, detections);
        vpiStreamSync(stream);

        VPIArrayData outData;
        vpiArrayLockData(detections, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &outData);
        
        // THE FINAL FIX: Access via .aos directly and dereference the sizePointer
        VPIAprilTagDetection *tagList = (VPIAprilTagDetection *)outData.buffer.aos.data;
        int numDetections = *outData.buffer.aos.sizePointer;

        for (int i = 0; i < numDetections; ++i) {
            if (tagList[i].id == 1) {
                std::cout << "Jupiter: Dock #1 Locked at X: " 
                          << tagList[i].center.x << " Y: " << tagList[i].center.y << std::endl;
            }
        }
        
        vpiArrayUnlock(detections);
        vpiImageDestroy(vpiFrame);

        cv::imshow("Jupiter Vision Feed", frame);
        if (cv::waitKey(10) == 'q') break;
    }

    vpiArrayDestroy(detections);
    vpiPayloadDestroy(payload);
    vpiStreamDestroy(stream);
    return 0;
}