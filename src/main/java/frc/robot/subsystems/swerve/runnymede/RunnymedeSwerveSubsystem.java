package frc.robot.subsystems.swerve.runnymede;


import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.Constants.Swerve.Module.BACK_LEFT;
import static frc.robot.Constants.Swerve.Module.BACK_RIGHT;
import static frc.robot.Constants.Swerve.Module.FRONT_LEFT;
import static frc.robot.Constants.Swerve.Module.FRONT_RIGHT;
import static frc.robot.Constants.Swerve.Motor.ANGLE;
import static frc.robot.Constants.Swerve.Motor.DRIVE;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

/**
 * Represents a swerve drive style drivetrain.
 */
public class RunnymedeSwerveSubsystem extends SwerveSubsystem {

    private final SwerveModule            frontLeft;
    private final SwerveModule            frontRight;
    private final SwerveModule            backLeft;
    private final SwerveModule            backRight;
    private final AHRS                    gyro;
    private Rotation3d                    gyroOffset;

    private final SwerveDriveKinematics   kinematics;
    public final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public RunnymedeSwerveSubsystem(HughVisionSubsystem visionSubsystem) {
        super(visionSubsystem);

        kinematics                    = new SwerveDriveKinematics(
            FRONT_LEFT.locationMetres,
            FRONT_RIGHT.locationMetres,
            BACK_LEFT.locationMetres,
            BACK_RIGHT.locationMetres);

        gyro                          = new AHRS(SerialPort.Port.kMXP);
        gyroOffset                    = gyro.getRotation3d();

        frontLeft                     = new SwerveModule(FRONT_LEFT, DRIVE, ANGLE);
        frontRight                    = new SwerveModule(FRONT_RIGHT, DRIVE, ANGLE);
        backLeft                      = new SwerveModule(BACK_LEFT, DRIVE, ANGLE);
        backRight                     = new SwerveModule(BACK_RIGHT, DRIVE, ANGLE);

        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            this.kinematics,
            gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            },
            new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
    }


    @Override
    protected void driveRawRobotOriented(ChassisSpeeds velocity) {

        // calculate desired states
        ChassisSpeeds       discretized        = ChassisSpeeds.discretize(velocity, Robot.kDefaultPeriod);
        Translation2d       centerOfRotation   = new Translation2d();
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(discretized, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, velocity,
            MAX_MODULE_SPEED_MPS, MAX_TRANSLATION_SPEED_MPS, MAX_ROTATIONAL_VELOCITY_PER_SEC.getRadians());

        // set states
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    @Override
    public void updateOdometryWithStates() {
        swerveDrivePoseEstimator.update(
            gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            });
    }

    @Override
    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    @Override
    protected void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        this.swerveDrivePoseEstimator.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
    }

    @Override
    public void zeroGyro() {
        gyroOffset = gyro.getRotation3d();
    }

    @Override
    public void lock() {
        // TODO: ADD SAFETY CODE

        // set speed to 0 and angle wheels to center
        frontLeft.setDesiredState(new SwerveModuleState(0.0, frontLeft.getPosition().angle));
        frontRight.setDesiredState(new SwerveModuleState(0.0, frontRight.getPosition().angle));
        backLeft.setDesiredState(new SwerveModuleState(0.0, backLeft.getPosition().angle));
        backRight.setDesiredState(new SwerveModuleState(0.0, backRight.getPosition().angle));

        // tell kinematics that we aren't moving
        kinematics.toSwerveModuleStates(new ChassisSpeeds());
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        this.swerveDrivePoseEstimator.resetPosition(gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            }, pose);
    }

    @Override
    public String toString() {
        return "Runnymede " + super.toString();
    }
}