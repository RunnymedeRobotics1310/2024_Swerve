package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE_RADIANS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveDistanceCommand extends BaseDriveCommand {

    private static final Translation2d DONT_MOVE    = new Translation2d(0, 0);

    private final Translation2d        velocityVectorMps;
    private final Double               distanceMetres;

    private Pose2d                     startingPose = null;
    private double                     travelled;
    private Rotation2d                 currentHeading;

    public DriveDistanceCommand(SwerveSubsystem swerve, Translation2d velocityVectorMps,
        Rotation2d heading, double distanceMetres) {
        super(swerve);
        this.velocityVectorMps = velocityVectorMps;
        setHeadingSetpoint(heading);
        this.distanceMetres = distanceMetres;
    }

    @Override
    public void initialize() {
        super.initialize();

        travelled      = 0;
        currentHeading = new Rotation2d();

        startingPose   = swerve.getPose();
        System.out.println("Drive/DistanceCommand Initialized at " + startingPose);

    }

    @Override
    public void execute() {
        super.execute();
        Translation2d currentLocation = swerve.getPose().getTranslation();
        travelled      = startingPose.getTranslation().getDistance(currentLocation);

        currentHeading = swerve.getPose().getRotation();

        Rotation2d omega = computeOmega(getHeadingSetpoint());

        if (wentTheDistance()) {
            swerve.driveFieldOriented(DONT_MOVE, omega);
        }
        else {
            swerve.driveFieldOriented(velocityVectorMps, omega);

        }
    }

    private boolean wentTheDistance() {

        double distErr = distanceMetres - travelled;

        return distErr <= 0;
    }

    private boolean lookingStraightAhead() {

        double headingErr = getHeadingSetpoint().getRadians() - currentHeading.getRadians();

        return Math.abs(headingErr) <= ROTATION_TOLERANCE_RADIANS;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        super.isFinished();

        boolean isFinished = wentTheDistance() && lookingStraightAhead();

        System.out.println("isFinished goal: " + distanceMetres + "m at " + getHeadingSetpoint().getDegrees() + " actual "
            + travelled + "m at " + currentHeading + ". done? " + isFinished);
        return isFinished;
    }
}
