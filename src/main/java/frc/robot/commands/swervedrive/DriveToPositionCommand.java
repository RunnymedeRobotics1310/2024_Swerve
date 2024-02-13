package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.subsystems.swerve.SwerveSubsystem.calculateVelocity;

public class DriveToPositionCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final Pose2d          desiredPose;

    public DriveToPositionCommand(SwerveSubsystem swerve, Pose2d pose) {
        this.swerve      = swerve;
        this.desiredPose = pose;
        addRequirements(swerve);
    }

    @Override
    public void execute() {

        Transform2d   transform = desiredPose.minus(swerve.getPose());

        Translation2d velocity  = calculateVelocity(transform.getTranslation(), 0);
        Rotation2d    omega     = swerve.computeOmega(desiredPose.getRotation());

        swerve.driveFieldOriented(velocity, omega);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerve.lock();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        super.isFinished();
        Transform2d transform = desiredPose.minus(swerve.getPose());
        boolean     transDone = Math.abs(transform.getTranslation().getNorm()) < TRANSLATION_TOLERANCE_METRES;
        boolean     rotatDone = Math.abs(transform.getRotation().getRadians()) < ROTATION_TOLERANCE_RADIANS;
        return transDone && rotatDone;
    }
}