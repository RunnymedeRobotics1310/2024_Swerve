package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveToPositionCommand extends BaseDriveCommand {

    private final Pose2d desiredPose;

    /**
     * Drive as fast as possible to the specified pose
     */
    public DriveToPositionCommand(SwerveSubsystem swerve, Pose2d pose) {
        super(swerve);
        this.desiredPose = pose;
    }

    @Override
    public void execute() {
        super.execute();
        driveToFieldPose(desiredPose);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerve.lock();
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return isCloseEnough(desiredPose.getTranslation()) && isCloseEnough(desiredPose.getRotation());
    }
}