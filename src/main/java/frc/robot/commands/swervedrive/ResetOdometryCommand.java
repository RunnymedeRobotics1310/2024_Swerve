package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ResetOdometryCommand extends InstantCommand {

    public ResetOdometryCommand(SwerveSubsystem swerve, Pose2d pose) {
        super(() -> {
            System.out.println("ResetOdometryCommand: Reset the pose to " + pose.toString());
            swerve.resetOdometry(pose);
        });
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}