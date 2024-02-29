package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.commands.test.SystemTestCommand.MotorOrModule.*;

public class SystemTestCommand extends LoggingCommand {

    enum MotorOrModule {
        NONE,
        FRONT_LEFT, BACK_LEFT, BACK_RIGHT, FRONT_RIGHT,
        AIM, LINK,
        SHOOTER_TOP, SHOOTER_BOTTOM,
        INTAKE,
        CLIMB_LEFT, CLIMB_RIGHT
    }

    private final OperatorInput   oi;
    private final SwerveSubsystem drive;

    private boolean               enabled       = false;
    private MotorOrModule         selectedMotor = NONE;
    private double                motorSpeed;
    private double                moduleDriveSpeed;
    private double                moduleRotationSpeed;

    public SystemTestCommand(OperatorInput oi, SwerveSubsystem drive) {
        this.oi    = oi;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        /*
         * The SystemTestCommand is not interruptible, and prevents all other commands that try to
         * interrupt it. Only the cancel button will end the SystemTestCommand.
         */
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void initialize() {
        super.initialize();
        stopAllMotors();
        enabled = true;
        updateDashboard();
    }


    @Override
    public void execute() {
        // todo: implement
    }

    public boolean isFinished() {

        // Wait 1/2 second before finishing.
        // This allows the user to start this command using the start and back
        // button combination without cancelling on the start button as
        // the user releases the buttons
        if (!isTimeoutExceeded(0.5d)) {
            return false;
        }

        // Cancel on the regular cancel button after the first 0.5 seconds
        if (oi.isCancel()) {
            setFinishReason("Cancelled by driver controller");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        stopAllMotors();
        enabled = false;
        updateDashboard();
        super.end(interrupted);
    }

    private void stopAllMotors() {
        motorSpeed          = 0;
        moduleDriveSpeed    = 0;
        moduleRotationSpeed = 0;
        drive.stop();
    }

    private void updateDashboard() {
        SmartDashboard.putBoolean("1310 Test Mode/Enabled", enabled);
        SmartDashboard.putString("1310 Test Mode/Motor or Module", selectedMotor.toString());
        SmartDashboard.putNumber("1310 Test Mode/Motor Speed", motorSpeed);
        SmartDashboard.putNumber("1310 Test Mode/Module Drive Speed", moduleDriveSpeed);
        SmartDashboard.putNumber("1310 Test Mode/Module Rotate Speed", moduleRotationSpeed);
    }

}
