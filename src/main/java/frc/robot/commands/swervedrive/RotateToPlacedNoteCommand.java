package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RunnymedeUtils;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RotateToPlacedNoteCommand extends BaseDriveCommand {

    private final Constants.BotTarget blueTarget;
    private final Constants.BotTarget redTarget;
    private Pose2d                    initialPose;
    private Constants.BotTarget       target = null;

    /**
     * Turn the robot to face the vision target specified
     *
     * @param swerve the swerve drive subsystem
     */
    public RotateToPlacedNoteCommand(SwerveSubsystem swerve, Constants.BotTarget blueTarget, Constants.BotTarget redTarget) {
        super(swerve);
        // this.hugh = hugh;
        this.blueTarget  = blueTarget;
        this.redTarget   = redTarget;
        this.initialPose = null;
        // addRequirements(hugh);

    }

    @Override
    public void initialize() {
        this.target = RunnymedeUtils.getRunnymedeAlliance() == DriverStation.Alliance.Blue ? blueTarget : redTarget;
        logCommandStart("Target: " + target);
        // hugh.setBotTarget(target);
        this.initialPose = swerve.getPose();
    }

    @Override
    public void execute() {
        super.execute();

        Translation2d robotRelativeTranslation = null;// jackman.getRobotTranslationToTarget();

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

        Translation2d robotRelativeTranslation = null; // jackman.getRobotTranslationToTarget();
        if (robotRelativeTranslation == null) {
            Rotation2d delta = getHeadingToFieldPosition(target.getLocation().toTranslation2d());
            return isCloseEnough(delta);
        }
        else {
            return Math.abs(robotRelativeTranslation.getAngle().getRadians()) <= ROTATION_TOLERANCE.getRadians();
        }

    }

}