package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.BotTarget;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.HughVisionSubsystem;


public class RotateToTargetCommand extends BaseDriveCommand {

    private final HughVisionSubsystem hugh;
    private final BotTarget           blueTarget;
    private final BotTarget           redTarget;
    private BotTarget                 target;
    private Pose2d                    initialPose;
    private final boolean             forwards;

    /**
     * Turn the robot to face the vision target specified
     * 
     * @param swerve the swerve drive subsystem
     * @param hugh the vision subsystem capable of seeing the target
     */
    public RotateToTargetCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh, BotTarget blueTarget, BotTarget redTarget,
        boolean forwards) {
        super(swerve);
        this.hugh        = hugh;
        this.blueTarget  = blueTarget;
        this.redTarget   = redTarget;
        this.target      = null;
        this.initialPose = null;
        this.forwards    = forwards;

        addRequirements(hugh);

    }

    @Override
    public void initialize() {
        if (getRunnymedeAlliance() == DriverStation.Alliance.Blue) {
            target = blueTarget;
        }
        else {
            target = redTarget;
        }
        logCommandStart("Target: " + target);
        hugh.setBotTarget(target);
        this.initialPose = swerve.getPose();
    }

    @Override
    public void execute() {
        super.execute();

        Rotation2d targetOffset = hugh.getTargetOffset();

        if (targetOffset == null) {
            Rotation2d delta = getHeadingToFieldPosition(target.getLocation().toTranslation2d());
            delta = delta.plus(Rotation2d.fromDegrees(180 * (forwards ? 1 : -1)));
            Rotation2d omega = computeOmega(delta);
            log("delta: " + delta + " omega: " + omega);
            swerve.driveFieldOriented(new Translation2d(), omega);
        }
        else {
            Rotation2d delta = targetOffset;
            delta = delta.plus(Rotation2d.fromDegrees(180 * (forwards ? 1 : -1)));
            Rotation2d omega = computeOmega(delta);
            log("delta: " + delta + " omega: " + omega);
            swerve.driveRobotOriented(new ChassisSpeeds(0, 0, omega.getRadians()));
        }

    }

    @Override
    public boolean isFinished() {

        Rotation2d targetOffset = hugh.getTargetOffset();
        if (targetOffset == null) {
            Rotation2d delta = getHeadingToFieldPosition(target.getLocation().toTranslation2d());
            delta = delta.plus(Rotation2d.fromDegrees(180 * (forwards ? 1 : -1)));

            return isCloseEnough(delta);
        }
        else {
            return Math.abs(targetOffset.getRadians()) <= ROTATION_TOLERANCE.getRadians();
        }

    }

}
