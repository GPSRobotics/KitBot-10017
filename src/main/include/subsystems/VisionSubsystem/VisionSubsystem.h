#pragma once

#include <frc2/command/SubsystemBase.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <frc/CameraServer.h>
#include <vector>

class VisionSubsystem : public frc2::SubsystemBase {
public:
    VisionSubsystem();

    void Periodic() override;

    // Check if a target is detected
    bool HasTarget() const;

    // Get the IDs of detected targets
    std::vector<int> GetTargetIDs() const;

    // Get the yaw offset of the target (in degrees)
    double GetTargetYaw() const;

    // Get the pitch offset of the target (in degrees)
    double GetTargetPitch() const;

private:
    cv::VideoCapture m_camera; // Camera object
    cv::Ptr<cv::aruco::Dictionary> m_dictionary; // AprilTag dictionary

    std::vector<int> m_targetIDs; // IDs of detected AprilTags
    double m_targetYaw = 0.0; // Yaw offset of the target
    double m_targetPitch = 0.0; // Pitch offset of the target
};