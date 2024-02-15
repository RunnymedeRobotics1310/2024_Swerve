package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putString("DriveToPosition/pose", "");
        SmartDashboard.putString("DriveToPosition/delta", "");
        SmartDashboard.putString("DriveToPosition/desired", format(desiredPose));
        SmartDashboard.putString("DriveToPosition/velocity", "");
    }

    @Override
    public void execute() {

        Pose2d        pose     = swerve.getPose();
        Transform2d   delta    = desiredPose.minus(pose);

        Translation2d velocity = calculateVelocity(delta.getTranslation());
        Rotation2d    omega    = swerve.computeOmega(desiredPose.getRotation());

        log("Pose: " + format(pose) + "  Delta: " + format(delta)
            + "  Velocity: " + format(velocity) + "m/s @ " + format(omega) + "/s");


        SmartDashboard.putString("DriveToPosition/pose", format(pose));
        SmartDashboard.putString("DriveToPosition/delta", format(delta));
        SmartDashboard.putString("DriveToPosition/desired", format(desiredPose));
        SmartDashboard.putString("DriveToPosition/velocity", format(velocity) + "m/s @ " + format(omega) + "/s");

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
        Transform2d transform  = desiredPose.minus(swerve.getPose());
        boolean     transDone  = Math.abs(transform.getTranslation().getNorm()) < TRANSLATION_TOLERANCE_METRES;
        boolean     rotateDone = Math.abs(transform.getRotation().getRadians()) < ROTATION_TOLERANCE_RADIANS;
        if (transDone && !rotateDone) {
            log("Translation complete");
        }
        if (rotateDone && !transDone) {
            log("Rotation complete");
        }
        return transDone && rotateDone;
    }
}