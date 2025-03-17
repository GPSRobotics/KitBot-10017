#pragma once

#include <units/length.h>
#include <units/angle.h>

namespace VisionConstants {
    // Camera settings
    constexpr int kCameraDeviceID = 0; // Device ID for the ArduCam
    constexpr int kCameraResolutionWidth = 640; // Width of the camera feed
    constexpr int kCameraResolutionHeight = 480; // Height of the camera feed

    // AprilTag settings
    constexpr int kAprilTagDictionary = cv::aruco::DICT_APRILTAG_36h11; // AprilTag dictionary
    constexpr double kFieldOfView = 60.0_deg; // Field of view of the camera (adjust as needed)

    // Offsets and scaling
    constexpr double kYawScalingFactor = 30.0; // Scaling factor for yaw offset
    constexpr double kPitchScalingFactor = 30.0; // Scaling factor for pitch offset
}