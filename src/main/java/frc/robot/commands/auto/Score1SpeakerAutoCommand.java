package frc.robot.commands.auto;

import static edu.wpi.first.wpilibj.DriverStation.getAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.auto.stubs.FakeScoreSpeakerCommand;
import frc.robot.commands.swervedrive.DriveDistanceCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.ResetOdometryCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class Score1SpeakerAutoCommand extends SequentialCommandGroup {

    public Score1SpeakerAutoCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {

        final Alliance  alliance = getAlliance().orElse(Alliance.Blue);
        final int       invert;
        final BotTarget speaker;
        final Pose2d    startingPose;
        final Pose2d    finishPose;



        if (alliance == Alliance.Blue) {
            startingPose = new Pose2d(0.63, 4, Rotation2d.fromDegrees(-28));
            speaker      = BotTarget.BLUE_SPEAKER;
            invert       = 1;
            finishPose   = new Pose2d(4, 1.5, new Rotation2d());
        }
        // Red Alliance
        else {
            startingPose = new Pose2d(15.91, 4, Rotation2d.fromDegrees(-28));
            speaker      = BotTarget.RED_SPEAKER;
            invert       = -1;
            finishPose   = new Pose2d(12.54, 1.5, new Rotation2d());

        }



        // TODO: implement Auto Selector
        // Configure
        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new ResetOdometryCommand(swerve, startingPose));

        /* ***AUTO PATTERN*** */

        /* Note 1 */
        addCommands(new DriveDistanceCommand(swerve, new Translation2d(invert * 0.5, 0), Rotation2d.fromDegrees(-28), 0.1));
        addCommands(new RotateToTargetCommand(swerve, hugh, speaker));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, finishPose));
        // addCommands(new DriveDistanceCommand(swerve, new Translation2d(invert * 1, 0),
        // Rotation2d.fromDegrees(0), 1));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}