package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static edu.wpi.first.wpilibj.DriverStation.*;
import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.commands.operator.OperatorInput.Axis.X;
import static frc.robot.commands.operator.OperatorInput.Axis.Y;
import static frc.robot.commands.operator.OperatorInput.Stick.LEFT;
import static frc.robot.commands.operator.OperatorInput.Stick.RIGHT;

public class TeleopDriveCommand extends LoggingCommand {

    private final SwerveSubsystem swerve;
    private final OperatorInput   oi;
    private Rotation2d            prevTheta = null;

    /**
     * Used to drive a swerve robot in full field-centric mode.
     */
    public TeleopDriveCommand(SwerveSubsystem swerve, OperatorInput operatorInput) {
        this.swerve = swerve;
        this.oi     = operatorInput;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // The FRC field-oriented ccoordinate system
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        Alliance alliance             = getAlliance().orElse(Alliance.Blue);

        // The coordinate system defines (0,0) as the right side of the blue alliance wall. The
        // x-axis is positive toward the red alliance, and the y-axis is positive to the left.
        // When the robot is on the red alliance, we need to invert inputs for the stick values
        // to move the robot in the right direction.
        int      invertFactor         = alliance == Alliance.Blue ? 1 : -1;

        // With the driver standing behind the driver station glass, "forward" on the left stick is
        // its y value, but that should convert into positive x movement on the field. The
        // Runnymede Controller inverts stick y-axis values, so "forward" on stick is positive.
        // Thus, positive y stick axis maps to positive x translation on the field.
        double   vX                   = oi.getDriverControllerAxis(LEFT, Y);

        // Left and right movement on the left stick (the stick's x-axis) maps to the y-axis on the
        // field. Left on the stick (negative x) maps to positive y on the field, and vice versa.
        // Thus, negative x stick axis maps to positive y translation on the field.
        double   vY                   = -oi.getDriverControllerAxis(LEFT, X);

        // Left and right on the right stick will change the direction the robot is facing - its
        // heading. Positive x values on the stick translate to clockwise motion, and vice versa.
        // The coordinate system has positive motion as CCW.
        // Therefore, negative x stick value maps to positive rotation on the field.
        double   ccwRotAngularVelPct  = -oi.getDriverControllerAxis(RIGHT, X);

        // User wants to jump directly to a specific heading. Computation is deferred because it is
        // complex
        // and may not be necessary. See below for details.
        int      rawDesiredHeadingDeg = oi.getPOV();

        // Compute boost factor
        boolean  isSlow               = oi.isDriverLeftBumper();
        boolean  isFast               = oi.isDriverRightBumper();
        double   boostFactor          = isSlow ? SLOW_SPEED_FACTOR : (isFast ? MAX_SPEED_FACTOR : GENERAL_SPEED_FACTOR);

        // write to dashboard
        SmartDashboard.putNumber("vX", vX);
        SmartDashboard.putNumber("vY", vY);
        SmartDashboard.putNumber("ccwRotAngularVelPct", ccwRotAngularVelPct);
        SmartDashboard.putNumber("rawDesiredHeadingDeg", rawDesiredHeadingDeg);
        SmartDashboard.putNumber("boostFactor", boostFactor);


        Translation2d translation = new Translation2d(
            Math.pow(vX, 3) * boostFactor * MAX_TRANSLATION_SPEED_MPS * invertFactor,
            Math.pow(vY, 3) * boostFactor * MAX_TRANSLATION_SPEED_MPS * invertFactor);

        Rotation2d    omega;

        // User is steering!
        if (ccwRotAngularVelPct != 0) {
            // Compute omega
            double w = Math.pow(ccwRotAngularVelPct, 3) * boostFactor * MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;
            omega     = Rotation2d.fromRadians(w);
            // Save previous heading for when we are finished steering.
            prevTheta = swerve.getPose().getRotation();
        }
        else if (rawDesiredHeadingDeg > -1) {
            // User wants to jump to POV
            // POV coordinates don't match field coordinates. POV is CW+ and field is CCW+. Also,
            // POV 0 is 90 degrees on the field (for blue alliance, and -90 for red).
            // Invert and rotate as required.
            // BLUE field = mod( (pov * -1) + 90 + 360, 360)
            // RED field = mod( (pov * -1) - 90 + 360, 360)
            double     correctedDegrees = ((rawDesiredHeadingDeg * -1) + (invertFactor * 90) + 360) % 360;
            Rotation2d desiredHeading   = Rotation2d.fromDegrees(correctedDegrees);
            omega     = swerve.computeOmega(desiredHeading);
            // Save the previous heading for when the jump is done
            prevTheta = desiredHeading;
        }
        else {
            // Translating only. If we know our previous heading, keep it and don't change it.
            // If we never set a specific heading before, just use the current one.
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
