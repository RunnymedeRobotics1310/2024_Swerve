package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.BotTarget;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE_RADIANS;

public class RotateToTargetCommand extends BaseDriveCommand {

    private final HughVisionSubsystem hugh;
    private final BotTarget           target;

    /**
     * Turn the robot to face the vision target specified
     * 
     * @param swerve the swerve drive subsystem
     * @param hugh the vision subsystem capable of seeing the target
     * @param target the target
     */
    public RotateToTargetCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh, BotTarget target) {
        super(swerve);
        this.hugh   = hugh;
        this.target = target;

        addRequirements(hugh);

    }

    @Override
    public void initialize() {
        super.initialize();
        hugh.setVisionTarget(target);
    }

    @Override
    public void execute() {
        super.execute();

        Translation2d robotRelativeTranslation = hugh.getRobotTranslationToTarget();

        if (robotRelativeTranslation == null) {
            swerve.driveFieldOriented(swerve.getPose().getTranslation(), target.getLocation().toTranslation2d().getAngle());
        }
        else {
            Rotation2d omega = computeOmega(robotRelativeTranslation.getAngle());
            swerve.driveRobotOriented(new ChassisSpeeds(0, 0, omega.getRadians()));
        }

    }

    @Override
    public boolean isFinished() {

        Translation2d robotRelativeTranslation = hugh.getRobotTranslationToTarget();
        if (robotRelativeTranslation == null) {
            return isCloseEnough(target.getLocation().toTranslation2d().getAngle());
        }
        else {
            return Math.abs(robotRelativeTranslation.getAngle().getRadians()) <= ROTATION_TOLERANCE_RADIANS;
        }

    }

}
