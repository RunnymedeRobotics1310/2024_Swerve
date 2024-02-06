package frc.robot.commands.swervedrive;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.Swerve.Chassis.MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;

public class TeleopDriveCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier  translationXSupplier, translationYSupplier;
    private final DoubleSupplier  rotationAngularVelocityPctSupplier;
    private final IntSupplier     jumpAngle;
    private final DoubleSupplier  boostFactor;

    private Rotation2d            previousHeading = null;

    /**
     * Used to drive a swerve robot in full field-centric mode. vX and vY supply
     * translation inputs, where x is
     * torwards/away from alliance wall and y is left/right. headingHorzontal and
     * headingVertical are the Cartesian
     * coordinates from which the robot's angle will be derived— they will be
     * converted to a polar angle, which the robot
     * will rotate to.
     *
     * @param swerve The swerve drivebase subsystem.
     * @param translationXSupplier DoubleSupplier that supplies the x-translation
     * joystick input. Should be in the range -1
     * to 1 with deadband already accounted for.
     * Positive X is away from the alliance wall.
     * @param translationYSupplier DoubleSupplier that supplies the y-translation
     * joystick input. Should be in the range -1
     * to 1 with deadband already accounted for.
     * Positive Y is towards the left wall when
     * looking through the driver station glass.
     * @param rotationAngularVelocityPctSupplier DoubleSupplier that supplies the
     * percentage of the maximum rotation
     * speed to be applied to the robot.
     * Should be in the range of -1 to
     * 1 with deadband already accounted
     * for. Positive values are CCW.
     */
    public TeleopDriveCommand(SwerveSubsystem swerve, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
        DoubleSupplier rotationAngularVelocityPctSupplier, IntSupplier jumpAngle, DoubleSupplier boostFactor) {
        this.swerve                             = swerve;
        this.translationXSupplier               = translationXSupplier;
        this.translationYSupplier               = translationYSupplier;
        this.rotationAngularVelocityPctSupplier = rotationAngularVelocityPctSupplier;
        this.jumpAngle                          = jumpAngle;
        this.boostFactor                        = boostFactor;

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
        double vX                         = translationXSupplier.getAsDouble();
        double vY                         = translationYSupplier.getAsDouble();
        double rotationAngularVelocityPct = rotationAngularVelocityPctSupplier.getAsDouble();
        int    desiredHeadingDegrees      = jumpAngle.getAsInt();
        double boostFactor                = this.boostFactor.getAsDouble();

        // write to dashboard
        SmartDashboard.putNumber("vX", vX);
        SmartDashboard.putNumber("vY", vY);
        SmartDashboard.putNumber("rotationAngularVelocityPct", rotationAngularVelocityPct);
        SmartDashboard.putNumber("jumpAngle", desiredHeadingDegrees);

        Translation2d vector = new Translation2d(
            Math.pow(vX, 3) * boostFactor * MAX_TRANSLATION_SPEED_MPS,
            Math.pow(vY, 3) * boostFactor * MAX_TRANSLATION_SPEED_MPS);

        Rotation2d    omega;

        // user is steering!
        if (rotationAngularVelocityPct != 0) {
            omega           = Rotation2d.fromRadians(Math.pow(rotationAngularVelocityPct, 3) * boostFactor
                * MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC);
            previousHeading = swerve.getPose().getRotation();

        }
        else if (desiredHeadingDegrees > -1) {
            // POV
            Rotation2d desiredHeading = Rotation2d.fromDegrees(desiredHeadingDegrees);
            previousHeading = desiredHeading;
            omega           = swerve.computeOmega(desiredHeading);
        }
        else {
            // translating only
            if (previousHeading == null) {
                previousHeading = swerve.getPose().getRotation();
            }
            omega = swerve.computeOmega(previousHeading);
        }

        System.out.println("TeleopDrive attempting to drive " + vector + "m/s " + omega + "rad/s");
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
