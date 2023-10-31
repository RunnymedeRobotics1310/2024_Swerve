package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveDriveConstants.SwerveModule;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.commands.operator.OperatorInput;
import frc.robot.commands.operator.RunnymedeGameController;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveTestCommand extends RunnymedeCommandBase {

    private final RunnymedeGameController driverController;
    private final SwerveDriveSubsystem    swerveDriveSubsystem;

    private SwerveModule                  swerveModule = SwerveModule.NONE;

    public SwerveDriveTestCommand(OperatorInput operatorInput, SwerveDriveSubsystem swerveDriveSubsystem) {

        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.driverController     = operatorInput.getRawDriverController();

        addRequirements(swerveDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        swerveModule = SwerveModule.NONE;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Check the bumpers and switch modules
        boolean leftBumperPressed  = driverController.getLeftBumperPressed();
        boolean rightBumperPressed = driverController.getRightBumperPressed();

        if (leftBumperPressed || rightBumperPressed) {

            // Stop the current module
            swerveDriveSubsystem.setMotorSpeeds(swerveModule, 0, 0);

            int index = 0;

            // Switch modules in a chain using the bumpers
            if (rightBumperPressed) {
                index = (swerveModule.ordinal() + 1) % SwerveModule.values().length;
            }
            else {
                index = swerveModule.ordinal() - 1;
                if (index < 0) {
                    index = SwerveModule.values().length - 1;
                }
            }

            swerveModule = SwerveModule.values()[index];
            return;
        }

        SmartDashboard.putString("Test Swerve Module", swerveModule.toString());

        switch (swerveModule) {
        case NONE:
            return;
        default:
            swerveDriveSubsystem.setMotorSpeeds(
                swerveModule, driverController.getLeftY(), driverController.getRightX());
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
