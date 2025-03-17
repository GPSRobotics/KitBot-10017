#include "subsystems/VisionSubsystem.h"
#include "constants.h"
#include <iostream>

VisionSubsystem::VisionSubsystem() {
    // Initialize the camera
    m_camera = cv::VideoCapture(VisionConstants::kCameraDeviceID);
    if (!m_camera.isOpened()) {
        std::cerr << "Error: Could not open camera!" << std::endl;
    }

    // Set the camera resolution
    m_camera.set(cv::CAP_PROP_FRAME_WIDTH, VisionConstants::kCameraResolutionWidth);
    m_camera.set(cv::CAP_PROP_FRAME_HEIGHT, VisionConstants::kCameraResolutionHeight);

    // Load the AprilTag dictionary
    m_dictionary = cv::aruco::getPredefinedDictionary(VisionConstants::kAprilTagDictionary);

    // Start the camera server for streaming
    frc::CameraServer::StartAutomaticCapture("ArduCam", VisionConstants::kCameraDeviceID);
}

void VisionSubsystem::Periodic() {
    cv::Mat frame;

    // Grab a frame from the camera
    if (m_camera.read(frame)) {
        // Detect AprilTags in the frame
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, m_dictionary, corners, m_targetIDs);

        // If tags are detected, calculate their offsets
        if (m_targetIDs.size() > 0) {
            // Calculate the center of the first detected tag
            cv::Point2f center = (corners[0][0] + corners[0][1] + corners[0][2] + corners[0][3]) / 4;

            // Calculate the horizontal and vertical offsets (yaw and pitch)
            m_targetYaw = (center.x - frame.cols / 2.0) / (frame.cols / 2.0) * VisionConstants::kYawScalingFactor;
            m_targetPitch = (center.y - frame.rows / 2.0) / (frame.rows / 2.0) * VisionConstants::kPitchScalingFactor;
        }
    }
}

bool VisionSubsystem::HasTarget() const {
    return !m_targetIDs.empty();
}

std::vector<int> VisionSubsystem::GetTargetIDs() const {
    return m_targetIDs;
}

double VisionSubsystem::GetTargetYaw() const {
    return m_targetYaw;
}

double VisionSubsystem::GetTargetPitch() const {
    return m_targetPitch;
}