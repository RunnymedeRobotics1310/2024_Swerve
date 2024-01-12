package frc.robot.commands.operator;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final RunnymedeGameController driverController;
    private final RunnymedeGameController operatorController;

    public enum Stick {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    /**
     * Construct an OperatorInput class that is fed by a DriverController and an OperatorController.
     *
     * @param driverControllerPort on the driver station which the driver joystick is plugged into
     * @param operatorControllerPort on the driver station which the aux joystick is plugged into
     */
    public OperatorInput(int driverControllerPort, int operatorControllerPort) {
        driverController   = new RunnymedeGameController(driverControllerPort);
        operatorController = new RunnymedeGameController(operatorControllerPort);
    }

    public OperatorInput(int driverControllerPort) {
        driverController   = new RunnymedeGameController(driverControllerPort);
        operatorController = null;
    }

    public boolean doHeadingCorrection() {
        return driverController.getRightBumper();
    }

    public boolean isZeroGyro() {
        return driverController.getStartButton();
    }

    public double jumpAngle() {
        return driverController.getPOV();
    }

    public double getDriverControllerAxis(Stick stick, Axis axis) {

        switch (stick) {

        case LEFT:
            switch (axis) {
            case X:
                return driverController.getLeftX();
            case Y:
                return driverController.getLeftY();
            }
            break;

        case RIGHT:
            switch (axis) {
            case X:
                return driverController.getRightX();
            case Y:
                return driverController.getRightY();
            }
            break;
        }

        return 0;
    }


    /**
     * return the raw underlying {@link RunnymedeGameController}. ONLY FOR USE IN TEST MODE.
     */
    public RunnymedeGameController getRawDriverController() {
        return driverController;
    }
}
