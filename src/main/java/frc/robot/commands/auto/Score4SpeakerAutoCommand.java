package frc.robot.commands.auto;

import static edu.wpi.first.wpilibj.DriverStation.getAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BotTarget;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class Score4SpeakerAutoCommand extends SequentialCommandGroup {

    public Score4SpeakerAutoCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {

        final Alliance  alliance = getAlliance().orElse(Alliance.Blue);
        final int       invert;
        final BotTarget wolverine;
        final BotTarget barnum;
        final BotTarget valjean;
        final BotTarget speaker;

        if (alliance == Alliance.Blue) {
            wolverine = BotTarget.BLUE_NOTE_WOLVERINE;
            barnum    = BotTarget.BLUE_NOTE_BARNUM;
            valjean   = BotTarget.BLUE_NOTE_VALJEAN;
            speaker   = BotTarget.BLUE_SPEAKER;
            invert    = 1;
        }
        // Red Alliance
        else {
            wolverine = BotTarget.RED_NOTE_WOLVERINE;
            barnum    = BotTarget.RED_NOTE_BARNUM;
            valjean   = BotTarget.RED_NOTE_VALJEAN;
            speaker   = BotTarget.RED_SPEAKER;
            invert    = -1;
        }



        // TODO: implement Auto Selector
        Rotation2d startingRotation = Rotation2d.fromDegrees(0);
        Pose2d     startingPose     = new Pose2d(1.37, 5.55, startingRotation);
        // Configure
        addCommands(new LogMessageCommand("Starting Auto"));

        addCommands(new ResetOdometryCommand(swerve, startingPose));

        /* ***AUTO PATTERN*** */

        /* Note 1 */
        // addCommands(new ScoreSpeakerCommand());

        addCommands(new DriveDistanceCommand(swerve, new Translation2d(invert * 0.25, 0), new Rotation2d(), 0.1));

        /* Note 2 */
        addCommands(new RotateToPlacedNoteCommand(swerve, wolverine));

        // addCommands(new VisionNotePickup());
        addCommands(new DriveRobotOrientedCommand(swerve, new Translation2d(.85, 0), new Rotation2d()));
        addCommands(new RotateToTargetCommand(swerve, hugh, speaker));
        // addCommands(new ScoreSpeakerCommand());


        /* Note 3 */
        addCommands(new RotateToPlacedNoteCommand(swerve, barnum));

        // addCommands(new VisionNotePickup());
        addCommands(new DriveRobotOrientedCommand(swerve, new Translation2d(.85, 0), new Rotation2d()));
        addCommands(new RotateToTargetCommand(swerve, hugh, speaker));
        // addCommands(new ScoreSpeakerCommand());

        /* Note 4 */
        addCommands(new RotateToPlacedNoteCommand(swerve, valjean));

        // addCommands(new VisionNotePickup());
        addCommands(new DriveRobotOrientedCommand(swerve, new Translation2d(.85, 0), new Rotation2d()));
        addCommands(new RotateToTargetCommand(swerve, hugh, speaker));
        // addCommands(new ScoreSpeakerCommand());

        /* Exit Zone */
        addCommands(new DriveToPositionCommand(swerve, new Pose2d(new Translation2d(7, 6.5), new Rotation2d())));
//        addCommands(new DriveDistanceCommand(swerve, new Translation2d(invert * 1, 0), Rotation2d.fromDegrees(0), 1));

        // tell people we're done
        addCommands(new LogMessageCommand("Auto Complete"));
    }
}
