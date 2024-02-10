package frc.robot.subsystems.swerve.yagsl;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;

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
    public YagslSubsystem(File configDirectory, VisionSubsystem visionSubsystem) {
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
    }

    @Override
    protected void driveRawRobotOriented(ChassisSpeeds velocity, Translation2d centerOfRotation) {
        swerveDrive.drive(velocity, false, centerOfRotation);
    }

    @Override
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

//    @Override
//    public Rotation2d getHeading() {
//        return swerveDrive.getYaw();
//    }

    @Override
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    @Override
    public void lock() {
        swerveDrive.lockPose();
    }

}
