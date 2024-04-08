package frc.robot.subsystems.swerve;

public record SwerveRotationConfig(
    double minRotVelocityRadPS,
    double maxRotVelocityRadPS,
    double maxJumpSpeedRadPS,
    double slowZoneRadians,
    double maxAccelerationRadPS2,
    double toleranceRadians,
    double headingP,
    double headingI,
    double headingD) {
}
