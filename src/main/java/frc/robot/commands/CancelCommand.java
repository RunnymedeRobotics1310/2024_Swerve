package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CancelCommand extends LoggingCommand {

    private final SwerveSubsystem      swerve;
    private static final Translation2d DONT_MOVE = new Translation2d();
    private static final Rotation2d    DONT_TURN = new Rotation2d();

    public CancelCommand(SwerveSubsystem swerve) {

        this.swerve = swerve;
        addRequirements(swerve);

    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        swerve.driveFieldOriented(DONT_MOVE, DONT_TURN);
    }
}