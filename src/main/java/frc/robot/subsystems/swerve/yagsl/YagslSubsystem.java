package frc.robot.subsystems.swerve.yagsl;

import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;

import java.io.File;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class YagslSubsystem extends SwerveSubsystem {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param configDirectory Directory of swerve drive config files.
     */
    public YagslSubsystem(File configDirectory, HughVisionSubsystem visionSubsystem) {
        super(visionSubsystem);
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(configDirectory).createSwerveDrive(MAX_TRANSLATION_SPEED_MPS);
        }
        catch (Exception e) {
            throw new RuntimeException(e);
        }
        // Runnymede does its own heading correction in the commands.
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    }

    @Override
    protected void driveRawRobotOriented(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity, false, new Translation2d());
    }

    @Override
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    @Override
    protected void updateOdometryWithStates() {
        // noop - done internally inside SwerveDrive
    }

    @Override
    protected void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        swerveDrive.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
    }

    @Override
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    @Override
    public void lock() {
        swerveDrive.lockPose();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    @Override
    public String toString() {
        return "YAGSL " + super.toString();
    }
}
