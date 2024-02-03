package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.commands.RunnymedeCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class DriveDistanceCommand extends RunnymedeCommand {

    private static final Translation2d DONT_MOVE = new Translation2d(0, 0);

    private final SwerveDriveSubsystem swerve;
    private final Translation2d velocityVectorMps;
    private final Rotation2d heading;
    private final Double distanceMetres;

    private Pose2d startingPose = null;
    private double travelled;
    private Rotation2d currentHeading;

    public DriveDistanceCommand(SwerveDriveSubsystem swerve, Translation2d velocityVectorMps,
            Rotation2d heading, double distanceMetres) {

        this.swerve = swerve;
        this.velocityVectorMps = velocityVectorMps;
        this.heading = heading;
        this.distanceMetres = distanceMetres;

    }

    @Override
    public void initialize() {
        super.initialize();

        travelled = 0;
        currentHeading = new Rotation2d();

        startingPose = swerve.getPose();
        System.out.println("DriveDistanceCommand Initialized at " + startingPose);

    }

    @Override
    public void execute() {

        Translation2d currentLocation = swerve.getPose().getTranslation();
        travelled = startingPose.getTranslation().getDistance(currentLocation);

        currentHeading = swerve.getHeading();

        Rotation2d omega = swerve.computeOmega(heading, currentHeading);

        if (wentTheDistance()) {
            swerve.driveFieldOriented(DONT_MOVE, omega);
        } else {
            swerve.driveFieldOriented(velocityVectorMps, omega);

        }
    }

    private boolean wentTheDistance() {

        double distErr = distanceMetres - travelled;

        return distErr <= 0;
    }

    private boolean lookingStraightAhead() {

        double headingErr = heading.getDegrees() - currentHeading.getDegrees();

        return Math.abs(headingErr) <= Constants.SwerveDriveConstants.ROTATION_TOLERANCE_DEGREES;
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

        System.out.println("isFinished goal: " + distanceMetres + "m at " + heading.getDegrees() + " actual "
                + travelled + "m at " + currentHeading + ". done? " + isFinished);
        return isFinished;
    }
}
