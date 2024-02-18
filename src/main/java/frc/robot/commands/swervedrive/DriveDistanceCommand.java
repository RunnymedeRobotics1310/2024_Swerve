package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE_RADIANS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveDistanceCommand extends BaseDriveCommand {

    private static final Translation2d DONT_MOVE   = new Translation2d(0, 0);

    private final Translation2d        velocityVectorMps;
    private final Double               distanceMetres;
    private final Rotation2d           headingSetpoint;
    private Pose2d                     desiredPose = null;

    public DriveDistanceCommand(SwerveSubsystem swerve, Translation2d velocityVectorMps,
        Rotation2d heading, double distanceMetres) {
        super(swerve);
        this.velocityVectorMps = velocityVectorMps;
        headingSetpoint        = heading;
        this.distanceMetres    = distanceMetres;
    }

    @Override
    public void initialize() {
        super.initialize();
        Rotation2d    angle       = velocityVectorMps.getAngle();
        Translation2d translation = new Translation2d(angle.getCos() * distanceMetres, angle.getSin() * distanceMetres);
        desiredPose = new Pose2d(swerve.getPose().getTranslation().plus(translation), headingSetpoint);
    }

    @Override
    public void execute() {
        super.execute();
        Rotation2d omega = computeOmega(headingSetpoint);
        if (isCloseEnough(desiredPose.getTranslation())) {
            swerve.driveFieldOriented(DONT_MOVE, omega);
        }
        else {
            swerve.driveFieldOriented(velocityVectorMps, omega);
        }
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
