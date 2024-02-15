package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig.P;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
import static frc.robot.Constants.VisionConstants.CAMERA_LOC_REL_TO_ROBOT_CENTER;
import static frc.robot.Constants.VisionConstants.getVisionStandardDeviation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionPositionInfo;
import frc.robot.subsystems.vision.VisionSubsystem;

public abstract class SwerveSubsystem extends SubsystemBase {

    private final VisionSubsystem visionSubsystem;

    private final SlewRateLimiter xLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter yLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(MAX_ROTATION_ACCELERATION_RAD_PER_SEC2);

    public SwerveSubsystem(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
    }

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
        double limitedX     = xLimiter.calculate(desiredVelocity.vxMetersPerSecond);
        double limitedY     = yLimiter.calculate(desiredVelocity.vyMetersPerSecond);
        double limitedOmega = omegaLimiter.calculate(desiredVelocity.omegaRadiansPerSecond);

        return new ChassisSpeeds(limitedX, limitedY, limitedOmega);
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
     * @param velocity the linear velocity of the robot in metres per second. Positive x is away
     * from the alliance wall, and positive y is toward the left wall when looking through the
     * driver station glass. Null means no translation.
     * @param omega the rotation rate of the heading of the robot. CCW positive. Null means no
     * rotation.
     * @param centerOfRotation the center of rotation of the robot, in case a center besides the
     * middle of the robot is desired. 0,0 represents the center of the robot.
     * @see #driveRobotOriented(ChassisSpeeds, Translation2d)
     */
    public final void driveFieldOriented(Translation2d velocity, Rotation2d omega, Translation2d centerOfRotation) {
        double        x             = velocity == null ? 0 : velocity.getX();
        double        y             = velocity == null ? 0 : velocity.getY();
        double        w             = omega == null ? 0 : omega.getRadians();
        Rotation2d    theta         = this.getPose().getRotation();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, w, theta);
        this.driveRobotOriented(chassisSpeeds, centerOfRotation);
    }

    /**
     * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
     * field-oriented inputs into the required robot-oriented {@link ChassisSpeeds} object that can
     * be used by the robot.
     *
     * @param velocity the linear velocity of the robot in metres per second. Positive x is away
     * from the alliance wall, and positive y is toward the left wall when looking through the
     * driver station glass.
     * @param omega the rotation rate of the heading of the robot, about the center of the robot.
     * CCW positive.
     * @see #driveFieldOriented(Translation2d, Rotation2d, Translation2d)
     */
    public final void driveFieldOriented(Translation2d velocity, Rotation2d omega) {
        this.driveFieldOriented(velocity, omega, new Translation2d());
    }

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
     * Lock the swerve drive to prevent it from moving.
     */
    public abstract void lock();

    /**
     * Utility function to compute the required rotation speed of the robot given its current
     * heading. Uses a PID controller to compute the offset.
     *
     * @param heading the desired heading of the robot
     * @return The required rotation speed of the robot
     * @see frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig
     */
    public final Rotation2d computeOmega(Rotation2d heading) {

        double error = (heading.getRadians() - getPose().getRotation().getRadians()) % 6;
        if (error > 3) {
            error -= 6;
        }
        double omega = (error * P) * MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;

        return Rotation2d.fromRadians(omega);
    }

    /**
     * Return a velocity that will traverse the specified translation as fast as possible without
     * overshooting the location. The initial speed is expected to be 0 and the final speed is
     * expected to be 0.
     *
     * @param translationToTravel the desired translation to travel
     * @return the velocity vector, in metres per second that the robot can safely travel
     * to traverse the distance specified
     */
    public static Translation2d calculateVelocity(Translation2d translationToTravel) {

        double distanceMetres = translationToTravel.getNorm();
        double sign           = Math.signum(distanceMetres);
        double absDistMetres  = Math.abs(distanceMetres);

        double maxSpeed       = MAX_TRANSLATION_SPEED_MPS;
        double decelDistance  = DECEL_FROM_MAX_TO_STOP_DIST_METRES;

        double decelDistRatio = absDistMetres / DECEL_FROM_MAX_TO_STOP_DIST_METRES;
        if (decelDistRatio < 1) {
            maxSpeed      = maxSpeed * decelDistRatio;
            decelDistance = decelDistance * decelDistRatio;
        }


        final double speed;

        if (absDistMetres >= decelDistance) {
            // cruising
            speed = sign * maxSpeed;
        }
        else {
            // decelerating
            double pctToGo = absDistMetres / decelDistance;
            speed = sign * maxSpeed * pctToGo;
        }

        Rotation2d angle = translationToTravel.getAngle();
        System.out.println("Need to travel " + translationToTravel + " at angle " + angle);
        return new Translation2d(speed * angle.getCos(), speed * angle.getSin());
    }

    abstract protected void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs);

    /**
     * Updates the field relative position of the robot using module
     * position data from the modules themselves, plus the gyro.
     */
    protected abstract void updateOdometryWithStates();

    /**
     * Update the field relative position of the robot using vision
     * position data returned from the vision subsystem.
     *
     * @see frc.robot.Constants.VisionConstants#getVisionStandardDeviation(double, double) for
     * tuning info
     */
    protected void updateOdometryWithVisionInfo() {
        VisionPositionInfo visPose = visionSubsystem.getPositionInfo();

        // ignore unreliable info from vision subsystem
        if (visPose == null) {
            return;
        }

        // convert camera pose to robot pose
        Pose2d         robotPose            = new Pose2d(visPose.pose().getTranslation().minus(CAMERA_LOC_REL_TO_ROBOT_CENTER),
            visPose.pose().getRotation());

        // how different is vision data from estimated data?
        double         poseDifferenceMetres = getPose().getTranslation().getDistance(robotPose.getTranslation());

        // todo: get confidence from VisionPositionInfo
        Matrix<N3, N1> stds                 = getVisionStandardDeviation(1.0, poseDifferenceMetres);

        // ignore drastically different data
        if (stds == null)
            return;

        // todo: determine if these are the right latencies to use
        double timestamp = Timer.getFPGATimestamp()
            - visPose.latencyMillis();

        System.out.println("Updating pose from vision: " + visPose.pose());
        this.addVisionMeasurement(visPose.pose(), timestamp, stds);
    }

    public abstract void resetOdometry(Pose2d replacementPose);

    @Override
    public void periodic() {
        super.periodic();
        updateOdometryWithStates();
        updateOdometryWithVisionInfo();
        Pose2d pose = getPose();
        SmartDashboard.putNumber("SwerveSubsystem/location/x", pose.getTranslation().getX());
        SmartDashboard.putNumber("SwerveSubsystem/location/y", pose.getTranslation().getY());
        SmartDashboard.putNumber("SwerveSubsystem/heading", pose.getRotation().getDegrees());
    }
}
