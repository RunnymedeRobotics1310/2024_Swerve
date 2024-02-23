package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.BotTarget;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;


public class RotateToTargetCommand extends BaseDriveCommand {

    private final HughVisionSubsystem hugh;
    private final BotTarget           target;
    private Pose2d                    initialPose;
    private final boolean             forwards;

    /**
     * Turn the robot to face the vision target specified
     * 
     * @param swerve the swerve drive subsystem
     * @param hugh the vision subsystem capable of seeing the target
     * @param target the target
     */
    public RotateToTargetCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh, BotTarget target, boolean forwards) {
        super(swerve);
        this.hugh        = hugh;
        this.target      = target;
        this.initialPose = null;
        this.forwards    = forwards;

        addRequirements(hugh);

    }

    @Override
    public void initialize() {
        logCommandStart("Target: " + target);
        hugh.setBotTarget(target);
        this.initialPose = swerve.getPose();
    }

    @Override
    public void execute() {
        super.execute();

        Translation2d robotRelativeTranslation = hugh.getRobotTranslationToTarget();

        if (robotRelativeTranslation == null) {
            Rotation2d delta = getHeadingToFieldPosition(target.getLocation().toTranslation2d());
            delta = delta.plus(Rotation2d.fromDegrees(180 * (forwards ? 1 : -1)));
            Rotation2d omega = computeOmega(delta);
            swerve.driveFieldOriented(new Translation2d(), omega);
        }
        else {
            Rotation2d delta = robotRelativeTranslation.getAngle();
            delta = delta.plus(Rotation2d.fromDegrees(180 * (forwards ? 1 : -1)));
            Rotation2d omega = computeOmega(delta);
            swerve.driveRobotOriented(new ChassisSpeeds(0, 0, omega.getRadians()));
        }

    }

    @Override
    public boolean isFinished() {

        Translation2d robotRelativeTranslation = hugh.getRobotTranslationToTarget();
        if (robotRelativeTranslation == null) {
            Rotation2d delta = getHeadingToFieldPosition(target.getLocation().toTranslation2d());
            delta = delta.plus(Rotation2d.fromDegrees(180 * (forwards ? 1 : -1)));

            return isCloseEnough(delta);
        }
        else {
            return Math.abs(robotRelativeTranslation.getAngle().getRadians()) <= ROTATION_TOLERANCE.getRadians();
        }

    }

}
