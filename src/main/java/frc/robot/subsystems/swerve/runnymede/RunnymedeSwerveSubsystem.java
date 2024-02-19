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
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Represents a swerve drive style drivetrain.
 */
public class RunnymedeSwerveSubsystem extends SwerveSubsystem {
    private final SwerveModule[]          modules;
    private final SwerveDriveKinematics   kinematics;
    private final AHRS                    gyro;
    private final SimulatedIMU            simulatedIMU;
    private Rotation3d                    gyroOffset;
    public Field2d                        field;

    public final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public RunnymedeSwerveSubsystem(HughVisionSubsystem visionSubsystem) {
        super(visionSubsystem);

        modules      = new SwerveModule[4];
        modules[0]   = new SwerveModule(FRONT_LEFT, DRIVE, ANGLE);
        modules[1]   = new SwerveModule(FRONT_RIGHT, DRIVE, ANGLE);
        modules[2]   = new SwerveModule(BACK_LEFT, DRIVE, ANGLE);
        modules[3]   = new SwerveModule(BACK_RIGHT, DRIVE, ANGLE);

        kinematics   = new SwerveDriveKinematics(
            Arrays.stream(modules).map(SwerveModule::getLocation).toArray(Translation2d[]::new));

        gyro         = new AHRS(SerialPort.Port.kMXP);
        gyroOffset   = gyro.getRotation3d();
        simulatedIMU = new SimulatedIMU();


        field        = new Field2d();
        SmartDashboard.putData(field);

        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            this.kinematics,
            gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
            Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new),
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
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    @Override
    public void updateOdometryWithStates() {
        swerveDrivePoseEstimator.update(
            gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
            Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new));

        Pose2d robotPose = swerveDrivePoseEstimator.getEstimatedPosition();

        field.setRobotPose(robotPose);

        if (RobotBase.isSimulation()) {
            simulatedIMU.updateOdometry(kinematics, getStates(), getModulePoses(robotPose), field);
        }
    }

    private SwerveModuleState[] getStates() {
        return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    @Override
    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    private Pose2d[] getModulePoses(Pose2d robotPose) {
        return Arrays.stream(modules).map(m -> {
            Transform2d tx = new Transform2d(m.getLocation(), m.getState().angle);
            return robotPose.plus(tx);
        }).toArray(Pose2d[]::new);
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
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(new SwerveModuleState(0.0, modules[i].getPosition().angle));
        }

        // tell kinematics that we aren't moving
        kinematics.toSwerveModuleStates(new ChassisSpeeds());
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        this.swerveDrivePoseEstimator.resetPosition(gyro.getRotation3d().minus(gyroOffset).toRotation2d(),
            Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new), pose);
    }

    @Override
    public String toString() {
        return "Runnymede " + super.toString();
    }
}