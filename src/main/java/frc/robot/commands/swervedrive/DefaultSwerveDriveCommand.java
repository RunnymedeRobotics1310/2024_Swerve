package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.operator.OperatorInput.Axis;
import frc.robot.commands.operator.OperatorInput.Stick;
import frc.robot.subsystems.SwerveDriveSubsystem;
import swervelib.SwerveController;

import java.util.function.BooleanSupplier;

public class DefaultSwerveDriveCommand extends RunnymedeCommandBase {

    private final SwerveDriveSubsystem swerve;
    private final OperatorInput operatorInput;
    private final BooleanSupplier isFieldRelativeDriveMode;
    private final boolean isOpenLoop;
    private final SwerveController controller;
    private final Timer timer = new Timer();
    private final boolean headingCorrection;
    private double angle = 0;
    private double lastTime = 0;


    /**
     * Creates a new ExampleCommand.
     *
     * @param swerve The subsystem used by this command.
     */
    public DefaultSwerveDriveCommand(OperatorInput operatorInput, SwerveDriveSubsystem swerve, BooleanSupplier isFieldRelativeDriveMode, boolean isOpenLoop, boolean headingCorrection) {
        this.operatorInput = operatorInput;
        this.swerve = swerve;
        this.isFieldRelativeDriveMode = isFieldRelativeDriveMode;
        this.isOpenLoop = isOpenLoop;
        this.controller = swerve.getSwerveController();
        this.headingCorrection = headingCorrection;
        if (headingCorrection) {
            timer.start();
        }
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (headingCorrection) {
            lastTime = timer.get();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xVelocity = Math.pow(operatorInput.getDriverControllerAxis(Stick.LEFT, Axis.X), 3);
        double yVelocity = Math.pow(operatorInput.getDriverControllerAxis(Stick.LEFT, Axis.Y), 3);
        double angVelocity = Math.pow(operatorInput.getDriverControllerAxis(Stick.RIGHT, Axis.X), 3);

        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("omega", angVelocity);

        if (headingCorrection) {
            // Estimate the desired angle in radians.
            angle += (angVelocity * (timer.get() - lastTime)) * controller.config.maxAngularVelocity;
            // Get the desired ChassisSpeeds given the desired angle and current angle.
            ChassisSpeeds correctedChassisSpeeds = controller.getTargetSpeeds(xVelocity, yVelocity, angle, swerve.getHeading().getRadians());
            // Drive using given data points.
            swerve.drive(SwerveController.getTranslation2d(correctedChassisSpeeds), correctedChassisSpeeds.omegaRadiansPerSecond, isFieldRelativeDriveMode.getAsBoolean(), isOpenLoop);
            lastTime = timer.get();
        } else {
            // Drive using raw values.
            swerve.drive(new Translation2d(xVelocity * controller.config.maxSpeed, yVelocity * controller.config.maxSpeed), angVelocity * controller.config.maxAngularVelocity, isFieldRelativeDriveMode.getAsBoolean(), isOpenLoop);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
