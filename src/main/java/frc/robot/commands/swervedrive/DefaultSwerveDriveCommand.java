package frc.robot.commands.swervedrive;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.RunnymedeCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DefaultSwerveDriveCommand extends RunnymedeCommand {

    private final SwerveDriveSubsystem swerve;
    private final DoubleSupplier translationXSupplier, translationYSupplier;
    private final DoubleSupplier rotationAngularVelocityPctSupplier;
    private final IntSupplier jumpAngle;
    private final DoubleSupplier boostFactor;

    private Rotation2d desiredJumpHeading = null;

    /**
     * Used to drive a swerve robot in full field-centric mode. vX and vY supply
     * translation inputs, where x is
     * torwards/away from alliance wall and y is left/right. headingHorzontal and
     * headingVertical are the Cartesian
     * coordinates from which the robot's angle will be derived— they will be
     * converted to a polar angle, which the robot
     * will rotate to.
     *
     * @param swerve               The swerve drivebase subsystem.
     * @param translationXSupplier DoubleSupplier that supplies the x-translation
     *                             joystick input. Should be in the range -1
     *                             to 1 with deadband already accounted for.
     *                             Positive X is away from the alliance wall.
     * @param translationYSupplier DoubleSupplier that supplies the y-translation
     *                             joystick input. Should be in the range -1
     *                             to 1 with deadband already accounted for.
     *                             Positive Y is towards the left wall when
     *                             looking through the driver station glass.
     */
    public DefaultSwerveDriveCommand(SwerveDriveSubsystem swerve, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationAngularVelocityPctSupplier, IntSupplier jumpAngle, DoubleSupplier boostFactor) {
        this.swerve = swerve;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationAngularVelocityPctSupplier = rotationAngularVelocityPctSupplier;
        this.jumpAngle = jumpAngle;
        this.boostFactor = boostFactor;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // get user inputs
        double vX = translationXSupplier.getAsDouble();
        double vY = translationYSupplier.getAsDouble();
        double rotationAngularVelocityPct = rotationAngularVelocityPctSupplier.getAsDouble();
        int desiredHeadingDegrees = jumpAngle.getAsInt();
        double boostFactor = this.boostFactor.getAsDouble();

        // write to dashboard
        SmartDashboard.putNumber("vX", vX);
        SmartDashboard.putNumber("vY", vY);
        SmartDashboard.putNumber("rotationAngularVelocityPct", rotationAngularVelocityPct);
        SmartDashboard.putNumber("jumpAngle", desiredHeadingDegrees);

        Translation2d vector = new Translation2d(
                Math.pow(vX, 3) * boostFactor * Constants.SwerveDriveConstants.MAX_SPEED_MPS,
                Math.pow(vY, 3) * boostFactor * Constants.SwerveDriveConstants.MAX_SPEED_MPS);


        // figure out the way we will set our heading
        final boolean jumpToPOV;
        if (desiredHeadingDegrees > -1) {
            // someone has pressed the POV. Jump to there.
            jumpToPOV = true;
            desiredJumpHeading = Rotation2d.fromDegrees(desiredHeadingDegrees);
        } else {
            // not pressing POV
            if (desiredJumpHeading != null) {
                // we still have a heading from before!
                if (Math.abs(
                        rotationAngularVelocityPct) < Constants.SwerveDriveConstants.ROTATION_ANGULAR_VELOCITY_TOLERANCE_PCT) {
                    // the user hasn't requested a turn so do not override the previous POV
                    // direction
                    jumpToPOV = true;
                } else {
                    // direction override - clear POV and follow turn override
                    jumpToPOV = false;
                    desiredJumpHeading = null;
                }
            } else {
                // we don't have a heading so we can't jump to the POV.
                jumpToPOV = false;
            }
        }

        final Rotation2d omega;
        // drive!
        if (jumpToPOV) {
            // jump
            Rotation2d currentHeading = swerve.getHeading();
            omega = swerve.computeOmega(desiredJumpHeading, currentHeading);

        } else {
            // steer

            // are we actually turning?
            if (Math.abs(
                    rotationAngularVelocityPct) > Constants.SwerveDriveConstants.ROTATION_ANGULAR_VELOCITY_TOLERANCE_PCT) {
                // yes!
                omega = Rotation2d.fromRadians(Math.pow(rotationAngularVelocityPct, 3) * Constants.SwerveDriveConstants.MAX_ROTATION_RADIANS_PER_SEC);
            } else {
                // no! rather than set "don't turn", give the exact heading
                omega = new Rotation2d();
            }
        }

        swerve.driveFieldOriented(vector, omega.getRadians());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        super.isFinished();
        return false;
    }
}
