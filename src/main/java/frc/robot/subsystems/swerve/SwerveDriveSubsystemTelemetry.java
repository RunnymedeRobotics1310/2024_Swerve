package frc.robot.subsystems.swerve;

import ca.team1310.swervedrive.telemetry.CoreSwerveDriveTelemetry;
import ca.team1310.swervedrive.telemetry.FieldAwareDriveTelemetry;
import ca.team1310.swervedrive.telemetry.VisionAwareDriveTelemetry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public record SwerveDriveSubsystemTelemetry(
    CoreSwerveDriveTelemetry coreSwerveDriveTelemetry,
    FieldAwareDriveTelemetry fieldAwareDriveTelemetry,
    VisionAwareDriveTelemetry visionAwareDriveTelemetry,
    Translation2d desiredFieldOrientedVelocity,
    Rotation2d desiredFieldOrientedRotation,
    Transform2d deltaToFieldPose) {
}
