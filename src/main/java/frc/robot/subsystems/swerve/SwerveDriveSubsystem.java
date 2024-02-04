package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveDriveSubsystem extends SubsystemBase {

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation {@link Translation2d} that is the commanded linear velocity of the robot,
     * in meters per second. In robot-relative mode, positive x is torwards the bow (front) and
     * positive y is torwards port (left). In field-relative mode, positive x is away from the
     * alliance wall (field North) and positive y is torwards the left wall when looking through the
     * driver stationglass (field West).
     * @param omega Robot angular rate. CCW positive.
     */
    public abstract void driveFieldOriented(Translation2d translation, Rotation2d omega);


    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public abstract Pose2d getPose();

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public abstract void zeroGyro();

    /**
     * Gets the current yaw angle of the robot, as reported by the imu. CCW
     * positive, not wrapped.
     *
     * @return The yaw angle
     */
    public abstract Rotation2d getHeading();

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public abstract void lock();

    public abstract Rotation2d computeOmega(Rotation2d heading, Rotation2d currentHeading);

}
