package frc.robot.commands.operator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.swervedrive.DriveDistanceCommand;
import frc.robot.commands.swervedrive.DriveToPositionCommand;
import frc.robot.commands.swervedrive.ResetOdometryCommand;
import frc.robot.commands.swervedrive.ZeroGyroCommand;
import frc.robot.commands.test.SystemTestCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.UsefulPoses.BLUE_2_2_20;
import static frc.robot.Constants.UsefulPoses.RED_2_2_20;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput {

    private final XboxController driverController;
    private final XboxController operatorController;

    public enum Stick {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    /**
     * Construct an OperatorInput class that is fed by a DriverController and an
     * OperatorController.
     *
     * @param driverControllerPort on the driver station which the driver joystick
     * is plugged into
     * @param operatorControllerPort on the driver station which the aux joystick is
     * plugged into
     */
    public OperatorInput(int driverControllerPort, int operatorControllerPort) {
        driverController   = new RunnymedeGameController(driverControllerPort);
        operatorController = new RunnymedeGameController(operatorControllerPort);
    }

    public boolean isToggleTestMode() {
        return !DriverStation.isFMSAttached() && driverController.getBackButton() && driverController.getStartButton();
    }

    public XboxController getRawDriverController() {
        return driverController;
    }


    public boolean isDriverLeftBumper() {
        return driverController.getLeftBumper();
    }

    public boolean isDriverRightBumper() {
        return driverController.getRightBumper();
    }

    public boolean isFaceSpeaker() {
        return driverController.getYButton();
    }

    public boolean isCancel() {
        return driverController.getStartButton();
    }

    public int getPOV() {
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
            }
            break;
        }

        return 0;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public void configureBindings(SwerveSubsystem driveSubsystem) {
        // Enter Test Mode (Start and Back pressed at the same time)
        new Trigger(() -> (isToggleTestMode()))
            .onTrue(
                new SystemTestCommand(this, driveSubsystem));

        new Trigger(driverController::getBackButton).onTrue(new ZeroGyroCommand(driveSubsystem));
        new Trigger(this::isCancel).whileTrue(new CancelCommand(driveSubsystem));
        new Trigger(driverController::getXButton)
            .whileTrue(new ResetOdometryCommand(driveSubsystem, new Pose2d(1.83, 0.40, Rotation2d.fromDegrees(0))));

        // drive forward
        Translation2d        fwd        = new Translation2d(0, 7);
        Rotation2d           fwdHeading = Rotation2d.fromDegrees(0);
        DriveDistanceCommand ddc        = new DriveDistanceCommand(driveSubsystem, fwd, fwdHeading, 3);
        new Trigger(driverController::getAButton).onTrue(ddc);

        // drive to position test
        Translation2d          location    = new Translation2d(2, 2);
        Rotation2d             heading     = Rotation2d.fromDegrees(-20);
        Pose2d                 desiredPose = new Pose2d(location, heading);
        DriveToPositionCommand dtpc        = new DriveToPositionCommand(driveSubsystem, BLUE_2_2_20, RED_2_2_20);
        new Trigger(driverController::getBButton).onTrue(dtpc);
    }
}