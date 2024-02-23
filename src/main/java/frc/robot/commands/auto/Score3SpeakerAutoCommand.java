package frc.robot.commands.auto;

import static edu.wpi.first.wpilibj.DriverStation.getAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.auto.stubs.FakeScoreSpeakerCommand;
import frc.robot.commands.auto.stubs.FakeVisionNotePickupCommand;
import frc.robot.commands.swervedrive.DriveDistanceCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.ResetOdometryCommand;
import frc.robot.commands.swervedrive.RotateToPlacedNoteCommand;
import frc.robot.commands.swervedrive.RotateToTargetCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class Score3SpeakerAutoCommand extends SequentialCommandGroup {

    public Score3SpeakerAutoCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {

        final Alliance   alliance         = getAlliance().orElse(Alliance.Blue);
        final int        invert;
        final BotTarget  barnum;
        final BotTarget  valjean;
        final BotTarget  speaker;
        final Pose2d     startingPose;
        final Rotation2d startingRotation = Rotation2d.fromDegrees(0);
        final Pose2d     finishPose;



        if (alliance == Alliance.Blue) {
            startingPose = new Pose2d(1.37, 5.55, startingRotation);
            barnum       = BotTarget.BLUE_NOTE_BARNUM;
            valjean      = BotTarget.BLUE_NOTE_VALJEAN;
            speaker      = BotTarget.BLUE_SPEAKER;
            invert       = 1;
            finishPose   = new Pose2d(new Translation2d(3.5, valjean.getLocation().getY()), new Rotation2d());
        }
        // Red Alliance
        else {
            startingPose = new Pose2d(15.17, 5.55, startingRotation);
            barnum       = BotTarget.RED_NOTE_BARNUM;
            valjean      = BotTarget.RED_NOTE_VALJEAN;
            speaker      = BotTarget.RED_SPEAKER;
            invert       = -1;
            finishPose   = new Pose2d(new Translation2d(13.04, valjean.getLocation().getY()), new Rotation2d());

        }

        // Configure
        addCommands(new LogMessageCommand("Starting Auto"));
        addCommands(new ResetOdometryCommand(swerve, startingPose));

        /* ***AUTO PATTERN*** */

        /* Note 1 */
        // back up to not hit the speaker while rotating
        addCommands(new DriveDistanceCommand(swerve, new Translation2d(invert * 0.5, 0), new Rotation2d(), 0.1));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Note 2 */
        addCommands(new RotateToPlacedNoteCommand(swerve, barnum));
        addCommands(new FakeVisionNotePickupCommand(swerve, barnum));
        addCommands(new RotateToTargetCommand(swerve, hugh, speaker, false));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Note 3 */
        addCommands(new RotateToPlacedNoteCommand(swerve, valjean));
        addCommands(new FakeVisionNotePickupCommand(swerve, valjean));
        addCommands(new RotateToTargetCommand(swerve, hugh, speaker, false));
        addCommands(new FakeScoreSpeakerCommand(swerve));

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, finishPose));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
