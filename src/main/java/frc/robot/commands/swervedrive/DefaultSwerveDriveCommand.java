package frc.robot.commands.swervedrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.RunnymedeCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class DefaultSwerveDriveCommand extends RunnymedeCommand {

    private final SwerveDriveSubsystem swerve;
    private final DoubleSupplier       vX, vY;
    private final DoubleSupplier       rotationAngularVelocity, jumpAngle;

    /**
     * Used to drive a swerve robot in full field-centric mode. vX and vY supply translation inputs, where x is
     * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
     * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
     * will rotate to.
     *
     * @param swerve The swerve drivebase subsystem.
     * @param vX DoubleSupplier that supplies the x-translation joystick input. Should be in the range -1
     * to 1 with deadband already accounted for. Positive X is away from the alliance wall.
     * @param vY DoubleSupplier that supplies the y-translation joystick input. Should be in the range -1
     * to 1 with deadband already accounted for. Positive Y is towards the left wall when
     * looking through the driver station glass.
     */
    public DefaultSwerveDriveCommand(SwerveDriveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
        DoubleSupplier rotationAngularVelocity,
        DoubleSupplier jumpAngle) {
        this.swerve            = swerve;
        this.vX                = vX;
        this.vY                = vY;
        this.rotationAngularVelocity   = rotationAngularVelocity;
        this.jumpAngle = jumpAngle;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double xVelocity   = Math.pow(vX.getAsDouble(), 3);
        double yVelocity   = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = -Math.pow(rotationAngularVelocity.getAsDouble(), 3);
        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("rotationAngularVelocity", angVelocity);

        Translation2d vector = new Translation2d(xVelocity * Constants.SwerveDriveConstants.MAX_SPEED_MPS, yVelocity * Constants.SwerveDriveConstants.MAX_SPEED_MPS);
        double rotation = angVelocity * Constants.SwerveDriveConstants.MAX_ROTATION_RADIANS_PER_SEC;
        // Drive using raw values.
        swerve.drive(vector, rotation, true);
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
