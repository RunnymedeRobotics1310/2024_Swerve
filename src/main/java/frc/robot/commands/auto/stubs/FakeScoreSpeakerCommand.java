package frc.robot.commands.auto.stubs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.swervedrive.DriveDistanceCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FakeScoreSpeakerCommand extends DriveDistanceCommand {
    public FakeScoreSpeakerCommand(SwerveSubsystem swerve) {
        super(swerve, new Translation2d(0.1, 0), Rotation2d.fromDegrees(180), 0.15);
    }
}
