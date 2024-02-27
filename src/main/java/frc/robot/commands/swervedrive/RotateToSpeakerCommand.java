package frc.robot.commands.swervedrive;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.BotTarget;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;


public class RotateToSpeakerCommand extends BaseDriveCommand {

    private Translation2d             speaker;
    private BotTarget                 speakerTarget;
    private final HughVisionSubsystem hugh;

    public RotateToSpeakerCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {
        super(swerve);
        this.hugh = hugh;
    }

    @Override
    public void initialize() {
        super.initialize();

        speakerTarget = getRunnymedeAlliance() == Blue
            ? Constants.BotTarget.BLUE_SPEAKER
            : Constants.BotTarget.RED_SPEAKER;

        speaker       = speakerTarget.getLocation().toTranslation2d();

        hugh.setBotTarget(speakerTarget);
    }

    @Override
    public void execute() {
        super.execute();
        // todo: use heading from hugh

        Rotation2d heading      = super.getHeadingToFieldPosition(speaker).plus(Rotation2d.fromDegrees(180));

        Rotation2d targetOffset = hugh.getTargetOffset();
        if (targetOffset == null) {
            System.out.println("no vision");
            // log("Heading to speaker: " + heading + " from location " +
            // swerve.getPose().getTranslation() + " for speaker " + speaker);
            Pose2d targetPose = new Pose2d(swerve.getPose().getTranslation(), heading);
            driveToFieldPose(targetPose);
        }
        else {
            System.out.println("vision");
            Rotation2d omega = computeOmega(targetOffset.plus(Rotation2d.fromDegrees(180)));
            swerve.driveRobotOriented(new ChassisSpeeds(0, 0, omega.getRadians()));
        }
    }

    @Override
    public boolean isFinished() {
        // todo: use heading from hugh
        Rotation2d targetOffset = hugh.getTargetOffset();
        if (targetOffset == null) {
            Rotation2d heading = super.getHeadingToFieldPosition(speaker).plus(Rotation2d.fromDegrees(180));
            return isCloseEnough(heading);
        }
        else {
            return Math.abs(targetOffset.getRadians() - Math.PI) <= ROTATION_TOLERANCE.getRadians();
        }
    }
}
