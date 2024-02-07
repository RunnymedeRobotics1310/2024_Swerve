package frc.robot.commands.swervedrive;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class LockCommand extends LoggingCommand {
    private final SwerveSubsystem swerve;

    public LockCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {

        swerve.lock();
    }
}
