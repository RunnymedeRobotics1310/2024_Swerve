package ca.team1310.swervedrive.odometry;

import ca.team1310.swervedrive.telemetry.GyroTelemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

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

    /**
     * Update the gyro in simulation mode. Not used in normal operation
     * 
     * @param kinematics
     * @param states
     * @param modulePoses
     * @param field
     */
    void updateOdometryForSimulation(SwerveDriveKinematics kinematics, SwerveModuleState[] states, Pose2d[] modulePoses,
        Field2d field);

    GyroTelemetry getTelemetryState();

    /**
     * Get the rotation of the robot as a Rotation2d object.
     */
    default Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }
}
