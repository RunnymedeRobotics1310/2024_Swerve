package frc.robot.commands.auto.stubs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.commands.swervedrive.BaseDriveCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.DriveToPositionFacingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class FakeVisionNotePickupCommand extends BaseDriveCommand {

    private final Translation2d blueLocation;
    private final Translation2d redLocation;
    private final Pose2d        bluePose;
    private final Pose2d        redPose;

    private Translation2d       location;
    private Rotation2d          heading;

    public FakeVisionNotePickupCommand(SwerveSubsystem swerve, Constants.BotTarget blueNote, Constants.BotTarget redNote) {
        super(swerve);
        this.bluePose     = null;
        this.redPose      = null;
        this.blueLocation = blueNote.getLocation().toTranslation2d();
        this.redLocation  = redNote.getLocation().toTranslation2d();
        this.heading      = null;
    }


    @Override
    public void initialize() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            if (bluePose == null) {
                location = blueLocation.minus(new Translation2d(.25, 0));
                heading  = swerve.getPose().getRotation();
            }
            else {
                location = bluePose.getTranslation().minus(new Translation2d(.25, 0));
                heading  = bluePose.getRotation();
            }
        }
        else {
            if (redPose == null) {
                location = redLocation.plus(new Translation2d(.25, 0));
                heading  = swerve.getPose().getRotation();
            }
            else {
                location = redPose.getTranslation().plus(new Translation2d(.25, 0));
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
        swerve.lock();
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return isCloseEnough(location) && isCloseEnough(heading);
    }
}
