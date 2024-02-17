package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionPositionInfo(Pose2d pose, double latencyMillis, PoseConfidence poseConfidence) {
    public String toString() {
        return String.format("%.2f,%.2f %.0fd %.0fms ", pose.getTranslation().getX(), pose.getTranslation().getY(),
            pose.getRotation().getDegrees(), latencyMillis) + poseConfidence;
    }
}

