package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionPositionInfo(Pose2d pose, double latencyMillis, PoseConfidence poseConfidence) {
}

