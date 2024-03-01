package frc.robot.commands.auto;


import static frc.robot.Constants.BotTarget.BLUE_NOTE_VALJEAN;
import static frc.robot.Constants.BotTarget.RED_NOTE_VALJEAN;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.auto.stubs.FakeScoreSpeakerCommand;
import frc.robot.commands.auto.stubs.FakeVisionNotePickupCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.RotateToPlacedNoteCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class Score3SpeakerAutoCommand extends SequentialCommandGroup {

    public Score3SpeakerAutoCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {


        Pose2d blueFinishPose = new Pose2d(new Translation2d(3.5, BLUE_NOTE_VALJEAN.getLocation().getY()),
            Rotation2d.fromDegrees(0));
        Pose2d redFinishPose  = new Pose2d(new Translation2d(13.04, RED_NOTE_VALJEAN.getLocation().getY()),
            Rotation2d.fromDegrees(180));


        addCommands(new LogMessageCommand("Starting Auto"));

        /* ***AUTO PATTERN*** */

        /* Note 1 */
        // back up to not hit the speaker while rotating
        addCommands(new DriveToPositionCommand(swerve,
            BotTarget.BLUE_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(1.6, 0)),
            BotTarget.RED_SPEAKER.getLocation().toTranslation2d().plus(new Translation2d(-1.6, 0))));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Note 3 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_BARNUM, BotTarget.RED_NOTE_BARNUM));
        addCommands(new FakeVisionNotePickupCommand(swerve, BotTarget.BLUE_NOTE_BARNUM, BotTarget.RED_NOTE_BARNUM));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Note 4 */
        addCommands(new RotateToPlacedNoteCommand(swerve, BotTarget.BLUE_NOTE_VALJEAN, BotTarget.RED_NOTE_VALJEAN));
        addCommands(new FakeVisionNotePickupCommand(swerve, BotTarget.BLUE_NOTE_VALJEAN, BotTarget.RED_NOTE_VALJEAN));
        addCommands(RotateToTargetCommand.createRotateToSpeakerCommand(swerve, hugh));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, blueFinishPose, redFinishPose));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
