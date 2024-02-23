package frc.robot.commands.auto.stubs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.swervedrive.DriveDistanceCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FakeScoreSpeakerCommand extends LoggingCommand {
    public FakeScoreSpeakerCommand(SwerveSubsystem swerve) {
        super();
    }

    @Override
    public void execute() {
        log("HE SHOOTS, HE SCORES!");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}