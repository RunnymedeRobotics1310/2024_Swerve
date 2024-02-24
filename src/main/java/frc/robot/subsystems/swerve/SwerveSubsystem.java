package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.Chassis.MAX_ROTATION_ACCELERATION_RAD_PER_SEC2;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_ACCELERATION_MPS2;
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
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import frc.robot.subsystems.vision.VisionPositionInfo;

public abstract class SwerveSubsystem extends SubsystemBase {

    private final HughVisionSubsystem visionSubsystem;

    private final SlewRateLimiter     xLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter     yLimiter     = new SlewRateLimiter(MAX_TRANSLATION_ACCELERATION_MPS2);
    private final SlewRateLimiter     omegaLimiter = new SlewRateLimiter(MAX_ROTATION_ACCELERATION_RAD_PER_SEC2);

    public SwerveSubsystem(HughVisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
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

        double x = velocity.vxMetersPerSecond;
        double y = velocity.vyMetersPerSecond;
        double w = velocity.omegaRadiansPerSecond;

        // Limit change in values. Note this may not scale evenly - one may reach desired
        // speed before another. This will be corrected the next time drive() is called.
        x = xLimiter.calculate(x);
        y = yLimiter.calculate(y);
        w = omegaLimiter.calculate(w);

        ChassisSpeeds safeVelocity = new ChassisSpeeds(x, y, w);

        SmartDashboard.putString("Drive/Swerve/chassis_robot", String.format("%.2f,%.2f m/s %.0f deg/s)",
            safeVelocity.vxMetersPerSecond, safeVelocity.vyMetersPerSecond,
            Rotation2d.fromRadians(safeVelocity.omegaRadiansPerSecond).getDegrees()));

        SmartDashboard.putString("Drive/Swerve/velocity_robot", String.format("%.2f m/s %.0f deg/s)",
            Math.hypot(safeVelocity.vxMetersPerSecond, safeVelocity.vyMetersPerSecond),
            Rotation2d.fromRadians(safeVelocity.omegaRadiansPerSecond).getDegrees()));

        driveRawRobotOriented(safeVelocity);
    }

    /**
     * The internal method for controlling the drivebase. This code does not apply any
     * limiters or validation, and should be used by implementing swerve drive subsystems
     * only.
     * <p>
     * Takes the desired chassis speeds of the robot - in a robot-oriented configuration.
     *
     * @param velocity The intended velocity of the robot chassis relative to itself.
     * @see ChassisSpeeds for how to construct a ChassisSpeeds object including
     * {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double, double, Rotation2d)}
     */
    protected abstract void driveRawRobotOriented(ChassisSpeeds velocity);

    /**
     * Convenience method for controlling the robot in field-oriented drive mode. Transforms the
     * field-oriented inputs into the required robot-oriented {@link ChassisSpeeds} object that can
     * be used by the robot.
     *
     * @param velocity the linear velocity of the robot in metres per second. Positive x is away
     * from the alliance wall, and positive y is toward the left wall when looking through the
     * driver station glass.
     * @param omega the rotation rate of the heading of the robot. CCW positive.
     * @see #driveRobotOriented(ChassisSpeeds)
     */
    public final void driveFieldOriented(Translation2d velocity, Rotation2d omega) {
        double     x     = velocity.getX();
        double     y     = velocity.getY();
        double     w     = omega.getRadians();
        Rotation2d theta = this.getPose().getRotation();
        SmartDashboard.putString("Drive/Swerve/velocity_field", LoggingCommand.format(velocity) + " m/s");

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, w, theta);
        this.driveRobotOriented(chassisSpeeds);
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
     * @see frc.robot.Constants.VisionConstants#getVisionStandardDeviation(frc.robot.subsystems.vision.PoseConfidence,
     * double) fortuning info
     */
    private void updateOdometryWithVisionInfo() {
        VisionPositionInfo visPose = visionSubsystem.getPositionInfo();

        // ignore unreliable info from vision subsystem
        if (visPose == null) {
            SmartDashboard.putString("Drive/Swerve/vispose", "");
            return;
        }

        // convert camera pose to robot pose
        Pose2d         robotPose = new Pose2d(visPose.pose().getTranslation().minus(CAMERA_LOC_REL_TO_ROBOT_CENTER),
            visPose.pose().getRotation());

        // how different is vision data from estimated data?
        double         delta_m   = getPose().getTranslation().getDistance(robotPose.getTranslation());

        Matrix<N3, N1> stds      = getVisionStandardDeviation(visPose.poseConfidence(), delta_m);

        // ignore drastically different data
        if (stds == null) {
            SmartDashboard.putString("Drive/Swerve/vispose", "");
            return;
        }

        double timeInSeconds = Timer.getFPGATimestamp() - (visPose.latencyMillis() / 1000);

        SmartDashboard.putString("Drive/Swerve/vispose", visPose.toString());
        this.addVisionMeasurement(visPose.pose(), timeInSeconds, stds);
    }

    public abstract void resetOdometry(Pose2d replacementPose);

    @Override
    public void periodic() {
        super.periodic();
        updateOdometryWithStates();
        updateOdometryWithVisionInfo();
        Pose2d pose = getPose();
        SmartDashboard.putString("Drive/Swerve/location",
            String.format("%.2f,%.2f m", pose.getTranslation().getX(), pose.getTranslation().getY()));
        SmartDashboard.putString("Drive/Swerve/heading", String.format("%.0f deg", pose.getRotation().getDegrees()));
    }

    @Override
    public String toString() {
        return "SwerveSubsystem Current Pose: " + LoggingCommand.format(getPose());
    }
}
