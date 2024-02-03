package frc.robot.subsystems.swerve.yagsl;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;

public class YagslSubsystem extends SwerveDriveSubsystem {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param configDirectory Directory of swerve drive config files.
     */
    public YagslSubsystem(File configDirectory) {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(configDirectory)
                    .createSwerveDrive(Constants.SwerveDriveConstants.MAX_SPEED_MPS);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot
        // via angle.

    }

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
    public void driveFieldOriented(Translation2d translation, double rotationRadiansPerSec) {
        ChassisSpeeds velocity = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotationRadiansPerSec, this.getPose().getRotation());
        swerveDrive.drive(velocity, false, new Translation2d());
    }



    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the imu. CCW
     * positive, not wrapped.
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    @Override
    public Rotation2d computeOmega(Rotation2d targetHeading, Rotation2d currentHeading) {
        SwerveController controller = swerveDrive.swerveController;
        double omegaRadiansPerSec = controller.headingCalculate(currentHeading.getRadians(), targetHeading.getRadians());
        return Rotation2d.fromRadians(omegaRadiansPerSec);
    }
}
