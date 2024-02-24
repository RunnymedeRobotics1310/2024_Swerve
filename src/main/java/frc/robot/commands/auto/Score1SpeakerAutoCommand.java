package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.auto.stubs.FakeScoreSpeakerCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.ResetOdometryCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class Score1SpeakerAutoCommand extends SequentialCommandGroup {

    public Score1SpeakerAutoCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {

        Pose2d blueFinishPose = new Pose2d(4, 1.5, new Rotation2d());
        Pose2d redFinishPose  = new Pose2d(12.54, 1.8, new Rotation2d());



        // TODO: implement Auto Selector
        // Configure
        addCommands(new LogMessageCommand("Starting Auto"));

        /* ***AUTO PATTERN*** */

        /* Note 1 */
        addCommands(new RotateToTargetCommand(swerve, hugh, BotTarget.BLUE_SPEAKER, BotTarget.RED_SPEAKER, false));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}