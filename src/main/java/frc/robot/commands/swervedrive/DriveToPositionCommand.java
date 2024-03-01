package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class DriveToPositionCommand extends BaseDriveCommand {

    private final Translation2d blueLocation;
    private final Translation2d redLocation;
    private final Pose2d        bluePose;
    private final Pose2d        redPose;

    private Translation2d       location;
    private Rotation2d          heading;

    public DriveToPositionCommand(SwerveSubsystem swerve, Translation2d blueLocation, Translation2d redLocation) {
        super(swerve);
        this.bluePose     = null;
        this.redPose      = null;
        this.blueLocation = blueLocation;
        this.redLocation  = redLocation;
        this.heading      = null;
    }

    /**
     * Drive as fast as possible to the specified pose
     */
    public DriveToPositionCommand(SwerveSubsystem swerve, Pose2d bluePose, Pose2d redPose) {
        super(swerve);
        this.bluePose     = bluePose;
        this.redPose      = redPose;
        this.blueLocation = null;
        this.redLocation  = null;
        this.heading      = null;
    }

    @Override
    public void initialize() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            if (bluePose == null) {
                location = blueLocation;
                heading  = swerve.getPose().getRotation();
            }
            else {
                location = bluePose.getTranslation();
                heading  = bluePose.getRotation();
            }
        }
        else {
            if (redPose == null) {
                location = redLocation;
                heading  = swerve.getPose().getRotation();
            }
            else {
                location = redPose.getTranslation();
                heading  = redPose.getRotation();
            }
        }
        logCommandStart("desiredPose: " + new Pose2d(location, heading));
    }

    @Override
    public void execute() {
        super.execute();
        driveToFieldPose(new Pose2d(location, heading));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return isCloseEnough(location) && isCloseEnough(heading);
    }
}