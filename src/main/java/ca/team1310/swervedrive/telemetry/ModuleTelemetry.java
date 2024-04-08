package ca.team1310.swervedrive.telemetry;

import edu.wpi.first.math.geometry.Translation2d;

public record ModuleTelemetry(
    String name,
    Translation2d location,
    double desiredSpeedMPS,
    double desiredAngleDegrees,
    double absoluteEncoderPositionDegrees,
    double angleMotorPositionDegrees,
    double driveMotorPositionMetres) {
}
