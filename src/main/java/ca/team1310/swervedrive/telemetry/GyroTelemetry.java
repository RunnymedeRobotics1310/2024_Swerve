package ca.team1310.swervedrive.telemetry;

public record GyroTelemetry(
    double rawImuDegrees,
    double adjustedImuDegrees) {
}
