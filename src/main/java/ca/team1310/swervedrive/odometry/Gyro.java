package ca.team1310.swervedrive.odometry;

import ca.team1310.swervedrive.telemetry.GyroTelemetry;
import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
    /**
     * Reset pitch, yaw, and roll to 0 degrees.
     */
    void zeroGyro();

    /**
     * Get the roll of the robot, in degrees.
     */
    double getRoll();

    /**
     * Get the pitch of the robot, in degrees.
     */
    double getPitch();

    /**
     * Get the yaw of the robot, in degrees.
     */
    double getYaw();

    GyroTelemetry getTelemetryState();

    /**
     * Get the rotation of the robot as a Rotation2d object.
     */
    default Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }
}
