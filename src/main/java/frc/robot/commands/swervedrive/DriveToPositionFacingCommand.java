package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;

public class DriveToPositionFacingCommand extends BaseDriveCommand {


    private final Translation2d positionToDriveToward;
    private final Translation2d positionToFace;

    /**
     * Drive as fast as possible to the specified location while facing another specified position.
     * Useful while driving one way and locking on another target.
     */
    public DriveToPositionFacingCommand(SwerveSubsystem swerve, Translation2d positionToDriveToward,
        Translation2d positionToFace) {
        super(swerve);
        this.positionToDriveToward = positionToDriveToward;
        this.positionToFace        = positionToFace;
    }

    @Override
    public void initialize() {
        logCommandStart("drive: " + format(positionToDriveToward) + " face: " + format(positionToFace));
    }

    @Override
    public void execute() {
        super.execute();
        Pose2d     current  = swerve.getPose();
        Rotation2d heading  = positionToFace.minus(current.getTranslation()).getAngle();
        Pose2d     nextPose = new Pose2d(positionToDriveToward, heading);
        driveToFieldPose(nextPose, MAX_TRANSLATION_SPEED_MPS);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        Rotation2d heading = getHeadingToFieldPosition(positionToFace);
        return isCloseEnough(positionToDriveToward) && isCloseEnough(heading);
    }
}