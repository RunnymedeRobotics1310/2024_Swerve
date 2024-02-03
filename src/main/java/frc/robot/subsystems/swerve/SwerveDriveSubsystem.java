package frc.robot.subsystems.swerve;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;

public abstract class SwerveDriveSubsystem extends SubsystemBase {

    /**
     * The primary method for controlling the drivebase. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation           {@link Translation2d} that is the commanded
     *                              linear velocity of the robot, in meters per
     *                              second. In robot-relative mode, positive x is
     *                              torwards the bow (front) and positive y is
     *                              torwards port (left). In field-relative mode,
     *                              positive x is away from the alliance wall
     *                              (field North) and positive y is torwards the
     *                              left wall when looking through the driver
     *                              station
     *                              glass (field West).
     * @param rotationRadiansPerSec Robot angular rate, in radians per second. CCW
     *                              positive. Unaffected by field/robot
     *                              relativity.
     */
    public abstract void driveFieldOriented(Translation2d translation, double rotationRadiansPerSec);

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public abstract void driveRobotOriented(ChassisSpeeds velocity);

    public abstract void driveFieldOriented(ChassisSpeeds velocity);



    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public abstract SwerveDriveKinematics getKinematics();

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public abstract void resetOdometry(Pose2d initialHolonomicPose);

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public abstract Pose2d getPose();

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public abstract void setChassisSpeeds(ChassisSpeeds chassisSpeeds);

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public abstract void postTrajectory(Trajectory trajectory);

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public abstract void zeroGyro();

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public abstract void setMotorBrake(boolean brake);

    /**
     * Gets the current yaw angle of the robot, as reported by the imu. CCW
     * positive, not wrapped.
     *
     * @return The yaw angle
     */
    public abstract Rotation2d getHeading();

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public abstract ChassisSpeeds getFieldVelocity();

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public abstract ChassisSpeeds getRobotVelocity();

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public abstract SwerveController getSwerveController();

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public abstract SwerveDriveConfiguration getSwerveDriveConfiguration();

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public abstract void lock() ;

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public abstract Rotation2d getPitch();

    /**
     * Add a fake vision reading for testing purposes.
     */
    public abstract void addFakeVisionReading();

    public abstract Rotation2d computeOmega(Rotation2d heading, Rotation2d currentHeading);

}
