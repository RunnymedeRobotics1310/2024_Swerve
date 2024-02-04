package frc.robot.commands.swervedrive;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DefaultSwerveDriveCommand extends LoggingCommand {

    private final SwerveDriveSubsystem swerve;
    private final DoubleSupplier translationXSupplier, translationYSupplier;
    private final DoubleSupplier rotationAngularVelocityPctSupplier;
    private final IntSupplier jumpAngle;
    private final DoubleSupplier boostFactor;

    private Rotation2d previousHeading = null;

    private Rotation2d desiredHeading = null;

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
     * @param rotationAngularVelocityPctSupplier DoubleSupplier that supplies the
     *                                           percentage of the maximum rotation
     *                                           speed to be applied to the robot.
     *                                           Should be in the range of -1 to
     *                                           1 with deadband already accounted
     *                                           for. Positive values are CCW.
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



        // compute heading mode
        if (desiredHeadingDegrees > -1) {
            // someone has pressed the POV. Jump to there.
            desiredHeading = Rotation2d.fromDegrees(desiredHeadingDegrees);
            // previous heading is not important.
            previousHeading = null;
        } else {
            // not pressing POV
            if (desiredHeading != null) {
                // we still have a heading from before!
                if (rotationAngularVelocityPct == 0) {
                    // the user hasn't requested a turn so do not override the previous POV direction
                } else {
                    // user is actively turning the robot. Clear desired heading.
                    desiredHeading = null;
                }
            } else {
                // user is not changing heading. keep it the same.
                if (previousHeading == null) {
                    // figure out what the heading is, so we can follow it
                    previousHeading = swerve.getPose().getRotation();
                }
                // our desired heading is the same as it was last iteration
                desiredHeading = previousHeading;
            }
        }



        final Rotation2d omega;

        // drive!
        if (desiredHeading != null) {
            // jump
            Rotation2d currentHeading = swerve.getHeading();
            omega = swerve.computeOmega(desiredHeading, currentHeading);
        } else {
            // steer
            omega = Rotation2d.fromRadians(Math.pow(rotationAngularVelocityPct, 3) * boostFactor * Constants.SwerveDriveConstants.MAX_ROTATION_RADIANS_PER_SEC);
        }

        swerve.driveFieldOriented(vector, omega);
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
