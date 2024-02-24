package frc.robot.commands.swervedrive;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;


public class RotateToSpeakerCommand extends BaseDriveCommand {

    private Translation2d             speaker;
    private final HughVisionSubsystem hugh;

    public RotateToSpeakerCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {
        super(swerve);
        this.hugh = hugh;
    }

    @Override
    public void initialize() {
        super.initialize();

        speaker = getRunnymedeAlliance() == Blue
            ? Constants.BotTarget.BLUE_SPEAKER.getLocation().toTranslation2d()
            : Constants.BotTarget.RED_SPEAKER.getLocation().toTranslation2d();
    }

    @Override
    public void execute() {
        super.execute();
        // todo: use heading from hugh

        Rotation2d    heading                  = super.getHeadingToFieldPosition(speaker).plus(Rotation2d.fromDegrees(180));

        Translation2d robotRelativeTranslation = hugh.getRobotTranslationToTarget();
        if (robotRelativeTranslation == null) {

            // log("Heading to speaker: " + heading + " from location " +
            // swerve.getPose().getTranslation() + " for speaker " + speaker);
            Pose2d targetPose = new Pose2d(swerve.getPose().getTranslation(), heading);
            driveToFieldPose(targetPose);
        }
        else {
            Rotation2d omega = computeOmega(robotRelativeTranslation.getAngle().plus(Rotation2d.fromDegrees(180)));
            swerve.driveRobotOriented(new ChassisSpeeds(0, 0, omega.getRadians()));
        }
    }

    @Override
    public boolean isFinished() {
        // todo: use heading from hugh
        Translation2d robotRelativeTranslation = hugh.getRobotTranslationToTarget();
        if (robotRelativeTranslation == null) {
            Rotation2d heading = super.getHeadingToFieldPosition(speaker).plus(Rotation2d.fromDegrees(180));
            return isCloseEnough(heading);
        }
        else {
            return Math.abs(robotRelativeTranslation.getAngle().getRadians()) <= ROTATION_TOLERANCE.getRadians();
        }
    }
}
