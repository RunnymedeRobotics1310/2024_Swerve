package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveRobotOrientedCommand extends BaseDriveCommand {

    private final Translation2d robotRelativeTranslation;
    private final Rotation2d    robotRelativeHeading;

    /**
     * Drive as fast as possible in the direciton specified at the heading specified
     * 
     * @param swerve
     * @param robotRelativeTranslation
     * @param robotRelativeHeading
     */
    public DriveRobotOrientedCommand(SwerveSubsystem swerve, Translation2d robotRelativeTranslation,
        Rotation2d robotRelativeHeading) {
        super(swerve);
        this.robotRelativeTranslation = robotRelativeTranslation;
        this.robotRelativeHeading     = robotRelativeHeading;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        Translation2d robotRelativeVelocity = computeVelocity(robotRelativeTranslation);
        // todo: implement
    }

    @Override
    public boolean isFinished() {
        // todo: implement
        return super.isFinished();
    }
}
