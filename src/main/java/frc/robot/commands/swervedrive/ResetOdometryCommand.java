package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

public class ResetOdometryCommand extends InstantCommand {

    public ResetOdometryCommand(SwerveSubsystem swerve, Pose2d bluePose, Pose2d redPose) {
        super(() -> {
            if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
                System.out.println("ResetOdometryCommand: Reset the pose to " + bluePose.toString());
                swerve.resetOdometry(bluePose);
            }
            else {
                System.out.println("ResetOdometryCommand: Reset the pose to " + redPose.toString());
                swerve.resetOdometry(redPose);
            }
        });
    }

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