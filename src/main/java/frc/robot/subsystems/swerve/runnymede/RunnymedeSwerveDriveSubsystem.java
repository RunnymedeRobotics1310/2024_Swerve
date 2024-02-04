package frc.robot.subsystems.swerve.runnymede;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.Constants.Swerve.Module.*;
import static frc.robot.Constants.Swerve.Motor.ANGLE;
import static frc.robot.Constants.Swerve.Motor.DRIVE;

/**
 * Represents a swerve drive style drivetrain.
 */
public class RunnymedeSwerveDriveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private Rotation3d gyroOffset;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    public RunnymedeSwerveDriveSubsystem() {

        kinematics = new SwerveDriveKinematics(
                FRONT_LEFT.locationMetres,
                FRONT_RIGHT.locationMetres,
                BACK_LEFT.locationMetres,
                BACK_RIGHT.locationMetres
        );

        gyro = new AHRS(SerialPort.Port.kMXP);
        gyroOffset = gyro.getRotation3d();

        frontLeft = new SwerveModule(FRONT_LEFT, DRIVE, ANGLE);
        frontRight = new SwerveModule(FRONT_RIGHT, DRIVE, ANGLE);
        backLeft = new SwerveModule(BACK_LEFT, DRIVE, ANGLE);
        backRight = new SwerveModule(BACK_RIGHT, DRIVE, ANGLE);

        odometry = new SwerveDriveOdometry(
                kinematics,
                gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }
        );

    }


    /**
     * Drive the robot according to the chassis speeds specified.
     *
     * @see ChassisSpeeds for how to calculate chassis speeds given omega
     * @see ChassisSpeedUtils for how to calculate chassis speeds using other inputs
     * @param desiredChassisSpeeds the desired chassis speed (including x, y, and rotation velocities)
     */
    public void drive(ChassisSpeeds desiredChassisSpeeds) {

        System.out.println("User requested drive at "+desiredChassisSpeeds);

        // calculate desired states
        ChassisSpeeds discretized = ChassisSpeeds.discretize(desiredChassisSpeeds, Robot.kDefaultPeriod);
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(discretized);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, desiredChassisSpeeds,
                MAX_MODULE_SPEED_MPS, MAX_TRANSLATION_SPEED_MPS, MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC);

        // set states
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        odometry.update(
                gyro.getRotation2d(),
                new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    /**
     * Get the pose from odometry
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Get the heading from the gyroscope
     */
    public Rotation2d getHeading() {
        return gyro.getRotation3d().minus(gyroOffset).toRotation2d();
    }

    public void zeroGyro() {
        gyroOffset = gyro.getRotation3d();
    }

    public void periodic() {
        updateOdometry();
    }
}