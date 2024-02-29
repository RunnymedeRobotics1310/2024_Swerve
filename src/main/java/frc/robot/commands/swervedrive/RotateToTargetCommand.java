package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE;
import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final boolean             forwards;
    int                               alignedCount = 0;


    public static RotateToTargetCommand createRotateToSpeakerCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh) {
        return new RotateToTargetCommand(swerve, hugh, BotTarget.BLUE_SPEAKER, BotTarget.RED_SPEAKER);
    }

    /**
     * Turn the robot to face the vision target specified
     * 
     * @param swerve the swerve drive subsystem
     * @param hugh the vision subsystem capable of seeing the target
     */
    private RotateToTargetCommand(SwerveSubsystem swerve, HughVisionSubsystem hugh, BotTarget blueTarget, BotTarget redTarget) {
        super(swerve);
        this.hugh       = hugh;
        this.blueTarget = blueTarget;
        this.redTarget  = redTarget;
        this.target     = null;
        this.forwards   = getForwards(blueTarget);

        addRequirements(hugh);
    }

    private boolean getForwards(BotTarget tgt) {

        if (tgt == BotTarget.BLUE_SPEAKER || tgt == BotTarget.BLUE_AMP) {
            return false;
        }
        else {
            return true;
        }
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
        alignedCount = 0;
    }


    @Override
    public void execute() {
        super.execute();

        Rotation2d targetOffset = hugh.getTargetOffset();
        if (targetOffset == null) {
            Rotation2d heading    = super.getHeadingToFieldPosition(target.getLocation().toTranslation2d())
                .plus(Rotation2d.fromDegrees(180 * (forwards ? 0 : 1)));
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
            Rotation2d heading = super.getHeadingToFieldPosition(target.getLocation().toTranslation2d())
                .plus(Rotation2d.fromDegrees(180 * (forwards ? 1 : -1)));
            return isCloseEnough(heading);
        }
        else {
            return Math.abs(targetOffset.getDegrees()) <= ROTATION_TOLERANCE.getDegrees();
        }
    }
}