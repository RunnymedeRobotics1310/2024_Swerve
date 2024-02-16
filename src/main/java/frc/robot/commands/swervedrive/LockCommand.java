package frc.robot.commands.swervedrive;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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
