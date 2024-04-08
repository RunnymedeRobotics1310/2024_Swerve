package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.Swerve.TRANSLATION_CONFIG;

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
        swerve.driveToFieldPose(nextPose, TRANSLATION_CONFIG.maxSpeedMPS());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        Rotation2d heading = swerve.getHeadingToFieldPosition(positionToFace);
        return swerve.isCloseEnough(positionToDriveToward) && swerve.isCloseEnough(heading);
    }
}