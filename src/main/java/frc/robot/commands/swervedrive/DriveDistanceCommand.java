package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveDistanceCommand extends BaseDriveCommand {

    private final Translation2d velocityVectorMps;
    private final Double        distanceMetres;
    private final Rotation2d    headingSetpoint;
    private Pose2d              desiredPose = null;

    public DriveDistanceCommand(SwerveSubsystem swerve, Translation2d velocityVectorMps,
        Rotation2d heading, double distanceMetres) {
        super(swerve);
        this.velocityVectorMps = velocityVectorMps;
        headingSetpoint        = heading;
        this.distanceMetres    = distanceMetres;
    }

    @Override
    public void initialize() {
        logCommandStart(
            "Velocity: " + velocityVectorMps + " headingSetpoint: " + headingSetpoint + " distance: " + distanceMetres + "m");
        Rotation2d    angle       = velocityVectorMps.getAngle();
        Translation2d translation = new Translation2d(angle.getCos() * distanceMetres, angle.getSin() * distanceMetres);
        desiredPose = new Pose2d(swerve.getPose().getTranslation().plus(translation), headingSetpoint);
    }

    @Override
    public void execute() {
        super.execute();
        driveToFieldPose(desiredPose, velocityVectorMps.getNorm());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return isCloseEnough(desiredPose.getTranslation()) && isCloseEnough(desiredPose.getRotation());
    }
}
