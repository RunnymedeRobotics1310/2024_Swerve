package ca.team1310.swervedrive.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public record FieldAwareDriveTelemetry(
    GyroTelemetry gyroState,
    Field2d field,
    Pose2d robotPose) {
}
