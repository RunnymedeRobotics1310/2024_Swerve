package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.Swerve.Chassis.*;

public class DriveToPositionCommand extends BaseDriveCommand {

    private final Pose2d     desiredPose;
    private final Rotation2d headingSetpoint;


    public DriveToPositionCommand(SwerveSubsystem swerve, Pose2d pose) {
        super(swerve);
        this.desiredPose = pose;
        headingSetpoint  = desiredPose.getRotation();
        SmartDashboard.putString("Drive/ToPosition/pose", "");
        SmartDashboard.putString("Drive/ToPosition/delta", "");
        SmartDashboard.putString("Drive/ToPosition/desired", format(desiredPose));
        SmartDashboard.putString("Drive/ToPosition/velocity", "");
    }

    @Override
    public void execute() {
        super.execute();

        Pose2d        pose     = swerve.getPose();
        Transform2d   delta    = desiredPose.minus(pose);

        Translation2d velocity = computeVelocity(delta.getTranslation());
        Rotation2d    omega    = computeOmega(headingSetpoint);

        log("Pose: " + format(pose) + "  Target: " + format(desiredPose) + "  Delta: " + format(delta)
            + "  Velocity: " + format(velocity) + "m/s @ " + format(omega) + "/s");


        SmartDashboard.putString("Drive/ToPosition/pose", format(pose));
        SmartDashboard.putString("Drive/ToPosition/delta", format(delta));
        SmartDashboard.putString("Drive/ToPosition/desired", format(desiredPose));
        SmartDashboard.putString("Drive/ToPosition/velocity", format(velocity) + "m/s @ " + format(omega) + "/s");

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