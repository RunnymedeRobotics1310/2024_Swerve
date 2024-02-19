package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE;

public class RotateToPlacedNoteCommand extends BaseDriveCommand {

    private final Constants.BotTarget target;
    private Pose2d                    initialPose;

    /**
     * Turn the robot to face the vision target specified
     *
     * @param swerve the swerve drive subsystem
     * @param target the target
     */
    public RotateToPlacedNoteCommand(SwerveSubsystem swerve, Constants.BotTarget target) {
        super(swerve);
//        this.hugh        = hugh;
        this.target      = target;
        this.initialPose = null;

//        addRequirements(hugh);

    }

    @Override
    public void initialize() {
        logCommandStart("Target: " + target);
//        hugh.setBotTarget(target);
        this.initialPose = swerve.getPose();
    }

    @Override
    public void execute() {
        super.execute();

        Translation2d robotRelativeTranslation = null;// hugh.getRobotTranslationToTarget();

        if (robotRelativeTranslation == null) {
            Rotation2d delta = getHeadingToFieldPosition(target.getLocation().toTranslation2d());
            Rotation2d omega = computeOmega(delta);
            swerve.driveFieldOriented(new Translation2d(), omega);
        }
        else {
            Rotation2d omega = computeOmega(robotRelativeTranslation.getAngle());
            swerve.driveRobotOriented(new ChassisSpeeds(0, 0, omega.getRadians()));
        }

    }

    @Override
    public boolean isFinished() {

        Translation2d robotRelativeTranslation = null; // hugh.getRobotTranslationToTarget();
        if (robotRelativeTranslation == null) {
            Rotation2d delta = getHeadingToFieldPosition(target.getLocation().toTranslation2d());
            return isCloseEnough(delta);
        }
        else {
            return Math.abs(robotRelativeTranslation.getAngle().getRadians()) <= ROTATION_TOLERANCE.getRadians();
        }

    }

}