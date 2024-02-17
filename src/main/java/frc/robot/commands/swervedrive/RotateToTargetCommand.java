package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.BotTarget;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;

public class RotateToTargetCommand extends BaseDriveCommand {

    private final HughVisionSubsystem hugh;
    private final BotTarget           target;

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
            Rotation2d delta   = swerve.getPose().getRotation().minus(target.getLocation().toTranslation2d().getAngle());
            double     degrees = Math.abs(delta.getDegrees());
            return degrees > 5.0;
        }
        else {
            double degrees = Math.abs(target.getLocation().toTranslation2d().getAngle().getDegrees());
            return degrees > 5.0;
        }

    }

}
