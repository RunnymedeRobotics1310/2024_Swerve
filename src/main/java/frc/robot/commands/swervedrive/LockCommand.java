package frc.robot.commands.swervedrive;

import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Lock the robot pose as long as this command is active (i.e. as long as the operator is pressing
 * the button.
 */
public class LockCommand extends BaseDriveCommand {

    public LockCommand(SwerveSubsystem swerve) {
        super(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        swerve.lock();
    }
}
