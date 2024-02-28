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
    int                               alignedCount = 0;

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
        alignedCount = 0;
    }

    @Override
    public void execute() {
        super.execute();

        Rotation2d targetOffset = hugh.getTargetOffset();
        if (targetOffset == null) {
            Rotation2d heading    = super.getHeadingToFieldPosition(speaker).plus(Rotation2d.fromDegrees(180));
            // log("Heading to speaker: " + heading + " from location " +
            // swerve.getPose().getTranslation() + " for speaker " + speaker);
            Pose2d     targetPose = new Pose2d(swerve.getPose().getTranslation(), heading);
            driveToFieldPose(targetPose);
        }
        else {
            // System.out.println("vision");
            Rotation2d omega = computeOmegaForOffset(targetOffset);
            System.out
                .println("offset: " + format(targetOffset) + " heading: " + format(swerve.getPose().getRotation()) + " omage: "
                    + format(omega));
            swerve.driveRobotOriented(new ChassisSpeeds(0, 0, omega.getRadians()));
        }
    }

    @Override
    public boolean isFinished() {
        // todo: use heading from hugh
        if (isAligned()) {
            alignedCount++;
        }
        else {
            alignedCount = 0;
        }

        return alignedCount >= 10;

    }

    private boolean isAligned() {
        Rotation2d targetOffset = hugh.getTargetOffset();
        if (targetOffset == null) {
            Rotation2d heading = super.getHeadingToFieldPosition(speaker).plus(Rotation2d.fromDegrees(180));
            return isCloseEnough(heading);
        }
        else {
            return Math.abs(targetOffset.getDegrees()) <= ROTATION_TOLERANCE.getDegrees();
        }
    }
}
