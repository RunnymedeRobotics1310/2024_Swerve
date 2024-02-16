package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public abstract class BaseDriveCommand extends LoggingCommand {
    protected final SwerveSubsystem swerve;

    private Rotation2d              lastSetTheta = Rotation2d.fromDegrees(0);

    public BaseDriveCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    protected final void setTheta(Rotation2d theta) {
        lastSetTheta = theta;
    }

    protected final Rotation2d getLastSetTheta() {
        return lastSetTheta;
    }
}
