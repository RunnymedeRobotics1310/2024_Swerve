package frc.robot.subsystems.swerve;

import ca.team1310.swervedrive.RunnymedeSwerveDrive;
import ca.team1310.swervedrive.utils.SwerveUtils;
import ca.team1310.swervedrive.vision.VisionAwareSwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static ca.team1310.swervedrive.utils.SwerveUtils.difference;
import static ca.team1310.swervedrive.utils.SwerveUtils.normalizeRotation;


public class SwerveSubsystem extends SubsystemBase {
    private final RunnymedeSwerveDrive       drive;
    private final SwerveDriveSubsystemConfig config;
    private final double                     robotPeriod;
    private final double                     maxTranslationSpeedMPS;
    private final SlewRateLimiter            xLimiter;
    private final SlewRateLimiter            yLimiter;
    private final SlewRateLimiter            omegaLimiter;
    private final PIDController              headingPIDController;
    private final PIDController              velocityPIDController;
    private Translation2d                    desiredFieldOrientedVelocity;
    private Rotation2d                       desiredFieldOrientedRotation;
    private Transform2d                      deltaToFieldPose;

    public SwerveSubsystem(SwerveDriveSubsystemConfig config) {
        this.drive                  = new VisionAwareSwerveDrive(config.coreConfig(), config.visionConfig());
        this.config                 = config;
        this.robotPeriod            = config.robotPeriod();
        this.maxTranslationSpeedMPS = config.coreConfig().maxAttainableTranslationSpeedMetresPerSecond();
        this.xLimiter               = new SlewRateLimiter(this.config.translationConfig().maxAccelMPS2());
        this.yLimiter               = new SlewRateLimiter(this.config.translationConfig().maxAccelMPS2());
        this.omegaLimiter           = new SlewRateLimiter(
            config.rotationConfig().maxAccelerationRadPS2());
        headingPIDController        = new PIDController(
            config.rotationConfig().headingP(),
            config.rotationConfig().headingI(),
            config.rotationConfig().headingD());
        velocityPIDController       = new PIDController(
            config.translationConfig().velocityP(),
            config.translationConfig().velocityI(),
            config.translationConfig().velocityD());
    }

    private void driveSafely(ChassisSpeeds robotOrientedVelocity) {

        ChassisSpeeds discretized = ChassisSpeeds.discretize(robotOrientedVelocity, robotPeriod);

        double        x           = discretized.vxMetersPerSecond;
        double        y           = discretized.vyMetersPerSecond;
        double        w           = discretized.omegaRadiansPerSecond;

        // Limit change in values. Note this may not scale
        // evenly - one may reach desired speed before another.

        // Use driveFieldOriented to avoid this.

        x = xLimiter.calculate(x);
        y = yLimiter.calculate(y);
        w = omegaLimiter.calculate(w);

        ChassisSpeeds safeVelocity = new ChassisSpeeds(x, y, w);

        if (this.config.enabled()) {
            this.drive.drive(safeVelocity);
        }
    }

    /**
     * The primary method for controlling the drivebase. The provided {@link ChassisSpeeds}
     * specifies the robot-relative chassis speeds of the robot.
     * <p>
     * This method is responsible for applying safety code to prevent the robot from attempting to
     * exceed its physical limits both in terms of speed and acceleration.
     *
     * @param velocity The intended velocity of the robot chassis relative to itself.
     * @see ChassisSpeeds for how to construct a ChassisSpeeds object including
     * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}
     */
    public final void driveRobotOriented(ChassisSpeeds velocity) {
        this.desiredFieldOrientedVelocity = new Translation2d();
        this.desiredFieldOrientedRotation = new Rotation2d();
        driveSafely(velocity);
    }

    /**
     * Stop all motors as fast as possible
     */
    public void stop() {
        driveRobotOriented(new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
     * field-oriented inputs into the required robot-oriented {@link ChassisSpeeds} object that can
     * be used by the robot.
     *
     * @param velocity the linear velocity of the robot in metres per second. Positive x is away
     * from the alliance wall, and positive y is toward the left wall when looking through the
     * driver station glass.
     * @param omega the rotation rate of the heading of the robot. CCW positive.
     */
    public final void driveFieldOriented(Translation2d velocity, Rotation2d omega) {
        this.desiredFieldOrientedVelocity = velocity;
        this.desiredFieldOrientedRotation = omega;

        double        x             = velocity.getX();
        double        y             = velocity.getY();
        double        w             = omega.getRadians();
        Rotation2d    theta         = drive.getPose().getRotation();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, w, theta);
        this.driveSafely(chassisSpeeds);
    }

    /**
     * Lock the swerve drive to prevent it from moving. This can only be called when the robot is
     * nearly stationary.
     *
     * @return true if successfully locked, false otherwise
     */
    public boolean lock() {
        return drive.lock();
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return drive.getPose();
    }

    /**
     * Return the gyro rotation for the robot, with yaw adjusted for the configured offset
     *
     * @return adjusted rotation3d from the gyro
     */
    public Rotation3d getGyroRotation3d() {
        return drive.getGyroRotation3d();
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        drive.zeroGyro();
    }

    public void resetOdometry(Pose2d pose) {
        drive.resetOdometry(pose);
    }


    /**
     * Set the desired module state for the named module. This should ONLY be used when testing
     * the serve drivebase in a controlled environment.
     *
     * This SHOULD NOT be called during normal operation - it is designed for TEST MODE ONLY!
     *
     * @param moduleName the module to activate
     * @param desiredState the state of the specified module.
     */
    public void setModuleState(String moduleName, SwerveModuleState desiredState) {
        drive.setModuleState(moduleName, desiredState);
    }

    @Override
    public void periodic() {
        drive.updateOdometry();
        postTelemetry();
    }

    public SwerveDriveSubsystemTelemetry getTelemetry() {
        return new SwerveDriveSubsystemTelemetry(
            drive.getCoreTelemetry(),
            drive.getFieldTelemetryState(),
            drive.getVisionTelemetry(),
            desiredFieldOrientedVelocity,
            desiredFieldOrientedRotation,
            deltaToFieldPose);
    }

    private void postTelemetry() {
        SwerveDriveSubsystemTelemetry telemetry = getTelemetry();
        // todo: implement. Consider by level
        SmartDashboard.putData(telemetry.fieldAwareDriveTelemetry().field());
    }

    @Override
    public String toString() {
        Pose2d pose  = getPose();
        double x     = pose.getX();
        double y     = pose.getY();
        double theta = pose.getRotation().getDegrees();
        return String.format("SwerveDriveSubsystem Pose: %.2f,%.2f @ %.1f deg", x, y, theta);
    }

    public boolean isCloseEnough(Rotation2d desiredHeading) {
        return SwerveUtils.isCloseEnough(drive.getPose().getRotation().getRadians(), desiredHeading.getRadians(),
            config.rotationConfig().toleranceRadians());
    }

    public boolean isCloseEnough(Translation2d desiredLocation) {
        return SwerveUtils.isCloseEnough(drive.getPose().getTranslation(), desiredLocation,
            config.translationConfig().toleranceMetres());
    }

    public boolean isCloseEnough(Pose2d desiredPose) {
        return isCloseEnough(desiredPose.getTranslation()) && isCloseEnough(desiredPose.getRotation());
    }

    public double getDistanceToFieldPositionMetres(Translation2d target) {
        return drive.getPose().getTranslation().getDistance(target);
    }

    /**
     * Compute the heading required to face the specified position on the field.
     *
     * @param target field position
     * @return the heading toward that position.
     */
    public Rotation2d getHeadingToFieldPosition(Translation2d target) {
        Translation2d currentRobotLocation = drive.getPose().getTranslation();
        Translation2d delta                = target.minus(currentRobotLocation);
        return delta.getAngle();
    }

    /**
     * Drive as fast as safely possible to the specified pose, up ot the max speed specified.
     *
     * @param desiredPose the desired location on the field
     */
    public final void driveToFieldPose(Pose2d desiredPose, double maxSpeedMPS) {
        Pose2d current = getPose();
        deltaToFieldPose             = difference(desiredPose, current);

        desiredFieldOrientedVelocity = computeVelocity(deltaToFieldPose.getTranslation(), maxSpeedMPS);
        desiredFieldOrientedRotation = computeOmega(desiredPose.getRotation());

        driveFieldOriented(desiredFieldOrientedVelocity, desiredFieldOrientedRotation);
    }

    /**
     * Return a velocity that will traverse the specified translation as fast as possible without
     * overshooting the location. The initial speed is expected to be 0 and the final speed is
     * expected to be 0.
     *
     * @param translationToTravel the desired translation to travel
     * @param maxSpeed the maximum speed to travel in Metres per Second
     * @return the velocity vector, in metres per second that the robot can safely travel
     * to traverse the distance specified
     */
    private Translation2d computeVelocity(Translation2d translationToTravel, double maxSpeed) {
        // todo: replace with PID
        double distanceMetres = translationToTravel.getNorm();

        // don't worry about tiny translations
        if (distanceMetres < config.translationConfig().toleranceMetres()) {
            return new Translation2d();
        }

        // safety code
        if (maxSpeed > maxTranslationSpeedMPS) {
            maxSpeed = maxTranslationSpeedMPS;
        }

        // ensure that we have enough room to decelerate
        double decelDistance  = Math.pow(maxTranslationSpeedMPS, 2)
            / (2 * config.translationConfig().maxAccelMPS2());
        double decelDistRatio = distanceMetres / decelDistance;
        if (decelDistRatio < 1) {
            maxSpeed *= decelDistRatio;
        }


        double speed;
        if (distanceMetres >= decelDistance) {
            // cruising
            speed = maxSpeed;
        }
        else {
            // decelerating
            double pctToGo = distanceMetres / decelDistance;
            speed = maxSpeed * pctToGo * velocityPIDController.getP();
        }

        // Confirm speed is not too slow to move
        if (speed < config.translationConfig().minSpeedMPS()) {
            speed = config.translationConfig().minSpeedMPS();
        }


        Rotation2d angle = translationToTravel.getAngle();

        double     xSign = Math.signum(translationToTravel.getX());
        double     ySign = Math.signum(translationToTravel.getY());
        return new Translation2d(xSign * speed * Math.abs(angle.getCos()), ySign * speed * Math.abs(angle.getSin()));
    }

    /**
     * Utility function to compute the required rotation speed of the robot given the heading
     * provided. Uses a PID controller to compute the offset.
     *
     * @param desiredHeading the desired heading of the robot
     * @return The required rotation speed of the robot
     */
    public Rotation2d computeOmega(Rotation2d desiredHeading) {
        // todo: replace with PID
        Pose2d currentPose = drive.getPose();
        double targetRad   = normalizeRotation(desiredHeading.getRadians());
        double currentRad  = normalizeRotation(currentPose.getRotation().getRadians());

        double errorRad    = targetRad - currentRad;
        errorRad = normalizeRotation(errorRad);
        double       absErrRad = Math.abs(errorRad);
        double       errSignum = Math.signum(errorRad);

        final double omegaRad;
        if (absErrRad < config.rotationConfig().toleranceRadians()) {
            omegaRad = 0;
        }
        else if (absErrRad < config.rotationConfig().slowZoneRadians()) {
            omegaRad = errSignum * config.rotationConfig().minRotVelocityRadPS();
        }
        else {
            omegaRad = errSignum * config.rotationConfig().maxJumpSpeedRadPS();
        }

        return Rotation2d.fromRadians(omegaRad);
    }
}