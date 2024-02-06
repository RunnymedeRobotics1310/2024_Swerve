package frc.robot.commands.swervedrive;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static edu.wpi.first.wpilibj.DriverStation.*;
import static frc.robot.Constants.Swerve.Chassis.MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;

public class TeleopDriveCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final DoubleSupplier  translationXSupplier, translationYSupplier;
    private final DoubleSupplier  rotationAngularVelocityPctSupplier;
    private final IntSupplier     jumpAngle;
    private final DoubleSupplier  boostFactor;

    private Rotation2d            prevTheta = null;

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

        // get raw user inputs
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

        // The FRC field-oriented ccoordinate system
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

        // The coordinate system defines 0,0 at the right side of the blue alliance wall. The
        // x axis is positive toward the red alliance, and the y axis is positive to the left.
        // When the robot is on the red alliance, we need to invert inputs for the stick values
        // to move the robot in the right direction.
        Alliance      alliance     = getAlliance().orElse(Alliance.Blue);
        int           invertFactor = alliance == Alliance.Blue ? 1 : -1;

        // With the driver standing behind the driver station glass, "forward" on the stick is
        // its y value, but that should convert into positive x movement on the field. The
        // Runnymede Controller inverts stick y axis values, so "forward" on stick is positive.
        // Thus, vY maps to positive x translation.
        // Left and right movement on the stick (the stick's x-axis) maps to the y-axis on the
        // field. "Left" is stick negative x, and that should map to positive y on the field,
        // and vice versa. Therefore, -vX maps to positive y translation.
        Translation2d translation  = new Translation2d(
            Math.pow(vY, 3) * boostFactor * MAX_TRANSLATION_SPEED_MPS * invertFactor,
            Math.pow(-vX, 3) * boostFactor * MAX_TRANSLATION_SPEED_MPS * invertFactor);

        Rotation2d    omega;

        // User is steering!
        if (rotationAngularVelocityPct != 0) {
            double w = Math.pow(rotationAngularVelocityPct, 3) * boostFactor * MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;
            omega     = Rotation2d.fromRadians(w);
            prevTheta = swerve.getPose().getRotation();

        }
        else if (desiredHeadingDegrees > -1) {
            // User wants to jump to POV
            // POV coordinates don't match field coordinates. POV is CW+ and field is CCW+. Also,
            // POV 0 is 90 degrees on the field (for blue alliance, and -90 for red).
            // Invert and rotate as required.
            // BLUE field = mod( (pov * -1) + 90 + 360, 360)
            // RED field = mod( (pov * -1) - 90 + 360, 360)
            double     correctedDegrees = ((desiredHeadingDegrees * -1) + (invertFactor * 90) + 360) % 360;
            Rotation2d desiredHeading   = Rotation2d.fromDegrees(correctedDegrees);
            prevTheta = desiredHeading;
            omega     = swerve.computeOmega(desiredHeading);
        }
        else {
            // translating only.
            if (prevTheta == null) {
                prevTheta = swerve.getPose().getRotation();
            }
            omega = swerve.computeOmega(prevTheta);
        }

        SmartDashboard.putString("Drive ", translation + " m/s " + omega);
        swerve.driveFieldOriented(translation, omega);

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
