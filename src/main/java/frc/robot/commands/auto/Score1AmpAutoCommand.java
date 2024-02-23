package frc.robot.commands.auto;

import static edu.wpi.first.wpilibj.DriverStation.getAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.ResetOdometryCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class Score1AmpAutoCommand extends SequentialCommandGroup {

    public Score1AmpAutoCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {

        final Alliance  alliance = getAlliance().orElse(Alliance.Blue);
        final int       invert;
        final BotTarget amp;
        final Pose2d    startingPose;
        final Pose2d    ampPose;
        final Pose2d    finishPose;


        if (alliance == Alliance.Blue) {
            startingPose = new Pose2d(1.5, 7.8, Rotation2d.fromDegrees(90));
            amp          = BotTarget.BLUE_AMP;
            ampPose      = new Pose2d(amp.getLocation().getX(), 7.8, Rotation2d.fromDegrees(90));
            invert       = 1;
            finishPose   = new Pose2d(4, 7.8, new Rotation2d(90));
        }
        // Red Alliance
        else {
            startingPose = new Pose2d(15.91, 7.8, Rotation2d.fromDegrees(90));
            amp          = BotTarget.RED_AMP;
            ampPose      = new Pose2d(amp.getLocation().getX(), 7.8, Rotation2d.fromDegrees(90));
            invert       = -1;
            finishPose   = new Pose2d(12.54, 7.8, new Rotation2d());

        }

        // TODO: implement Auto Selector
        // Configure
        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new ResetOdometryCommand(swerve, startingPose));


        addCommands(new DriveToPositionCommand(swerve, ampPose));
        // addCommands(new ScoreAmpCommand(swerve));
        addCommands(new DriveToPositionCommand(swerve, finishPose));

        addCommands(new LogMessageCommand("Auto Complete"));

    }
}
