package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.Chassis.MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;
import static frc.robot.Constants.Swerve.Chassis.MAX_ROTATION_ACCELERATION_RAD_PER_SEC2;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_ACCELERATION_MPS2;
import static frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig.P;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveSubsystem extends SubsystemBase {

    private final SlewRateLimiter xLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter yLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(MAX_ROTATION_ACCELERATION_RAD_PER_SEC2);

    /**
     * The primary method for controlling the drivebase. The provided {@link ChassisSpeeds}
     * specifies
     * the robot-relative chassis speeds of the robot. The provided {@link Translation2d} specifies
     * the center of rotation of the robot.
     * <p>
     * This method is responsible for applying safety code to prevent the robot from attempting to
     * exceed its physical limits both in terms of speed and acceleration.
     *
     * @param velocity The intended velocity of the robot chassis relative to itself.
     * @param centerOfRotation The center of rotation of the robot's rotation, in metres. 0,0 is the
     * center of the robot.
     * @see ChassisSpeeds for how to construct a ChassisSpeeds object including
     * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}
     */
    public final void driveRobotOriented(ChassisSpeeds velocity, Translation2d centerOfRotation) {
        ChassisSpeeds safeVelocity = getSafeChassisSpeeds(velocity);
        driveRawRobotOriented(safeVelocity, centerOfRotation);
    }

    /**
     * Safety code - make sure that the input chassis speeds aren't too different from the
     * last measured chassis speeds.
     * 
     * @param desiredVelocity desired velocity
     * @return safe velocity
     */
    private ChassisSpeeds getSafeChassisSpeeds(ChassisSpeeds desiredVelocity) {
        // Limit change in values. Note this may not scale evenly - one may reach desired
        // speed before another. This will be corrected the next time drive() is called.
        double        limitedX     = xLimiter.calculate(desiredVelocity.vxMetersPerSecond);
        double        limitedY     = yLimiter.calculate(desiredVelocity.vyMetersPerSecond);
        double        limitedOmega = omegaLimiter.calculate(desiredVelocity.omegaRadiansPerSecond);

        // limit
        ChassisSpeeds safeVelocity = new ChassisSpeeds(limitedX, limitedY, limitedOmega);

        // report
//        if (desiredVelocity.vxMetersPerSecond != safeVelocity.vxMetersPerSecond
//            || desiredVelocity.vyMetersPerSecond != safeVelocity.vyMetersPerSecond
//            || desiredVelocity.omegaRadiansPerSecond != safeVelocity.omegaRadiansPerSecond) {
//            System.out.println("Velocity limited by SlewRateLimiters. Desired: " + desiredVelocity + ", actual: " + safeVelocity);
//        }
        return safeVelocity;
    }

    /**
     * The primary method for controlling the drivebase. The provided {@link ChassisSpeeds}
     * specifies
     * the robot-relative chassis speeds of the robot. The provided {@link Translation2d} specifies
     * the center of rotation of the robot.
     *
     * @param velocity The intended velocity of the robot chassis relative to itself.
     * @param centerOfRotation The center of rotation of the robot's rotation, in metres. 0,0 is the
     * center of the robot.
     * @see ChassisSpeeds for how to construct a ChassisSpeeds object including
     * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}
     */
    protected abstract void driveRawRobotOriented(ChassisSpeeds velocity, Translation2d centerOfRotation);

    /**
     * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
     * field-oriented inputs into the required robot-oriented {@link ChassisSpeeds} object that can
     * be used by the robot.
     *
     * @param translation the linear velocity of the robot in metres per second. Positive x is away
     * from the alliance wall, and positive y is toward the left wall when looking through the
     * driver station glass.
     * @param omega the rotation rate of the heading of the robot. CCW positive.
     * @param centerOfRotation the center of rotation of the robot, in case a center besides the
     * middle of the robot is desired. 0,0 represents the center of the robot.
     * @see #driveRobotOriented(ChassisSpeeds, Translation2d)
     */
    public final void driveFieldOriented(Translation2d translation, Rotation2d omega, Translation2d centerOfRotation) {
        double        x        = translation.getX();
        double        y        = translation.getY();
        double        w        = omega.getRadians();
        Rotation2d    theta    = this.getPose().getRotation();
        ChassisSpeeds velocity = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, w, theta);
        this.driveRobotOriented(velocity, centerOfRotation);
    }

    /**
     * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
     * field-oriented inputs into the required robot-oriented {@link ChassisSpeeds} object that can
     * be used by the robot.
     *
     * @param translation the linear velocity of the robot in metres per second. Positive x is away
     * from the alliance wall, and positive y is toward the left wall when looking through the
     * driver station glass.
     * @param omega the rotation rate of the heading of the robot, about the center of the robot.
     * CCW positive.
     * @see #driveFieldOriented(Translation2d, Rotation2d, Translation2d)
     */
    public final void driveFieldOriented(Translation2d translation, Rotation2d omega) {
        this.driveFieldOriented(translation, omega, new Translation2d());
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public abstract Pose2d getPose();

    /**
     * Gets the current yaw angle of the robot, as reported by the imu. CCW
     * positive, not wrapped.
     *
     * @return The yaw angle
     */
    public abstract Rotation2d getHeading();

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public abstract void zeroGyro();

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public abstract void lock();

    /**
     * Utility function to compute the required rotation speed of the robot given its current
     * heading. Uses a PID controller to compute the offset.
     *
     * @see frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig
     * @param heading the desired heading of the robot
     * @return The required rotation speed of the robot
     */
    public final Rotation2d computeOmega(Rotation2d heading) {

        double error = (heading.getRadians() - getPose().getRotation().getRadians()) % 6;
        if (error > 3) {
            error -= 6;
        }
        double omega = (error * P) * MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;

        return Rotation2d.fromRadians(omega);
    }

}
