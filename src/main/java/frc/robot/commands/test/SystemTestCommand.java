package frc.robot.commands.test;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.commands.test.SystemTestCommand.Motor.*;

public class SystemTestCommand extends LoggingCommand {

    enum Motor {
        NONE,
        FRONT_LEFT_DRIVE, FRONT_LEFT_TURN,
        BACK_LEFT_DRIVE, BACK_LEFT_TURN,
        BACK_RIGHT_DRIVE, BACK_RIGHT_TURN,
        FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURN,
        AIM, LINK,
        SHOOTER_TOP, SHOOTER_BOTTOM,
        INTAKE,
        CLIMB_LEFT, CLIMB_RIGHT
    }

    private final OperatorInput   oi;
    private final XboxController  controller;
    private final SwerveSubsystem drive;


    private boolean               enabled             = false;
    private Motor                 selectedMotor       = NONE;
    private double                motorSpeed;

    private boolean               previousLeftBumper  = false;
    private boolean               previousRightBumper = false;

    public SystemTestCommand(OperatorInput oi, SwerveSubsystem drive) {
        this.oi         = oi;
        this.controller = oi.getRawDriverController();
        this.drive      = drive;
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
        readSelectedMotor();
        readMotorSpeed();
        applyMotorSpeed();
        updateDashboard();
    }


    /**
     * Use the bumpers to select the next / previous motor in the motor ring.
     * <p>
     * Switching motors will cause all motors to stop
     */
    private void readSelectedMotor() {

        boolean rightBumper = controller.getRightBumper() && !previousRightBumper;
        previousRightBumper = controller.getRightBumper();

        boolean leftBumper = controller.getLeftBumper() && !previousLeftBumper;
        previousLeftBumper = controller.getLeftBumper();

        if (rightBumper || leftBumper) {

            int nextMotorIndex = selectedMotor.ordinal();

            if (rightBumper) {

                // Select the next motor in the ring
                nextMotorIndex = (nextMotorIndex + 1) % Motor.values().length;
            }
            else {

                // Select the previous motor in the ring
                nextMotorIndex--;
                if (nextMotorIndex < 0) {
                    nextMotorIndex = Motor.values().length - 1;
                }
            }

            stopAllMotors();

            selectedMotor = Motor.values()[nextMotorIndex];

        }
    }


    /**
     * The SystemTestCommand can use either the POV or the triggers to control
     * the motor speed. If the triggers are used, the POV is cleared.
     * <p>
     * Once the motor is selected, use the POV up and down to
     * adjust the motor speed.
     * <p>
     * The speed is adjusted 50 times / second as the user holds the
     * POV control. Allow 5 seconds to ramp the speed from 0 to full value.
     * <p>
     * increment = 1.0 (full) / 50 adjustments/sec / 5 sec = .004 adjustment size / loop.
     */
    private void readMotorSpeed() {

        int    pov          = controller.getPOV();
        double leftTrigger  = controller.getLeftTriggerAxis();
        double rightTrigger = controller.getRightTriggerAxis();


        if (controller.getXButton()) {
            // If the X button is pressed, reset the motor speed to zero
            motorSpeed = 0;
        }
        else if (leftTrigger > 0 && rightTrigger > 0) {

            // If both triggers are pressed, then stop the motor
            motorSpeed = 0;
        }
        else if (leftTrigger > 0) {

            motorSpeed = -leftTrigger;
        }
        else if (rightTrigger > 0) {

            motorSpeed = rightTrigger;
        }
        else {

            // No triggers are pressed, use the POV to control the motor speed
            if (pov == 0) {

                motorSpeed += 0.004;

                if (motorSpeed > 1.0) {
                    motorSpeed = 1.0;
                }
            }

            if (pov == 180) {

                motorSpeed -= 0.004;

                if (motorSpeed < -1.0) {
                    motorSpeed = -1.0;
                }
            }

        }

    }

    /**
     * Apply the selected motor speed to the selected motor
     */
    private void applyMotorSpeed() {
        switch (selectedMotor) {
        case NONE:
            break;
        }
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
        motorSpeed = 0;
        drive.stop();
    }

    private void updateDashboard() {
        SmartDashboard.putBoolean("1310 Test Mode/Enabled", enabled);
        SmartDashboard.putString("1310 Test Mode/Motor or Module", selectedMotor.toString());
        SmartDashboard.putNumber("1310 Test Mode/Motor Speed", motorSpeed);
    }

}
