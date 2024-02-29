package frc.robot.commands.test;

import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SystemTestCommand extends LoggingCommand {

    private enum MotorOrModule {
        FRONT_LEFT, BACK_LEFT, BACK_RIGHT, FRONT_RIGHT,
        AIM, LINK,
        SHOOTER_TOP, SHOOTER_BOTTOM,
        INTAKE,
        CLIMB_LEFT, CLIMB_RIGHT
    }

    private final OperatorInput   oi;
    private final SwerveSubsystem drive;

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
        super.end(interrupted);
    }

    private void stopAllMotors() {
        drive.stop();
    }
}
