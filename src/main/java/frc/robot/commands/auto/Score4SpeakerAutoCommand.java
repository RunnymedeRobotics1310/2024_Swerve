package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.auto.stubs.FakeScoreSpeakerCommand;
import frc.robot.commands.auto.stubs.FakeVisionNotePickupCommand;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class Score4SpeakerAutoCommand extends SequentialCommandGroup {

    public Score4SpeakerAutoCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {

        final Pose2d blueStartingPose = new Pose2d(1.37, 5.55, Rotation2d.fromDegrees(0));
        final Pose2d redStartingPose  = new Pose2d(15.17, 5.55, Rotation2d.fromDegrees(180));

        final Pose2d blueFinishPose   = new Pose2d(new Translation2d(3.5, 7), new Rotation2d());
        final Pose2d redFinishPose    = new Pose2d(new Translation2d(13.04, 7), new Rotation2d());


        // TODO: implement Auto Selector
        // Configure
        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new ResetOdometryCommand(swerve, blueStartingPose, redStartingPose));

        /* ***AUTO PATTERN*** */

        /* Note 1 */
        // back up to not hit the speaker while rotating
        addCommands(new DriveToPositionCommand(swerve,
            blueStartingPose.plus(new Transform2d(.25, 0, new Rotation2d())),
            redStartingPose.plus(new Transform2d(.25, 0, new Rotation2d()))));
        addCommands(new FakeScoreSpeakerCommand(swerve));


        /* Note 2 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_WOLVERINE, BotTarget.RED_NOTE_WOLVERINE));
        addCommands(new FakeVisionNotePickupCommand(swerve, BotTarget.BLUE_NOTE_WOLVERINE, BotTarget.RED_NOTE_WOLVERINE));
        addCommands(new RotateToSpeakerCommand(swerve, hugh));
        addCommands(new FakeScoreSpeakerCommand(swerve));


        /* Note 3 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_BARNUM, BotTarget.RED_NOTE_BARNUM));
        addCommands(new FakeVisionNotePickupCommand(swerve, BotTarget.BLUE_NOTE_BARNUM, BotTarget.RED_NOTE_BARNUM));
        addCommands(new RotateToSpeakerCommand(swerve, hugh));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Note 4 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_VALJEAN, BotTarget.RED_NOTE_VALJEAN));
        addCommands(new FakeVisionNotePickupCommand(swerve, BotTarget.BLUE_NOTE_VALJEAN, BotTarget.RED_NOTE_VALJEAN));
        addCommands(new RotateToSpeakerCommand(swerve, hugh));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
