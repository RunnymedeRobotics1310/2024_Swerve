package frc.robot.commands.swervedrive;

import static frc.robot.Constants.Swerve.Chassis.DECEL_FROM_MAX_TO_STOP_DIST_METRES;
import static frc.robot.Constants.Swerve.Chassis.MAX_ROTATIONAL_VELOCITY_PER_SEC;
import static frc.robot.Constants.Swerve.Chassis.MAX_TRANSLATION_SPEED_MPS;
import static frc.robot.Constants.Swerve.Chassis.MIN_ROTATIONAL_VELOCITY_PER_SEC;
import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE;
import static frc.robot.Constants.Swerve.Chassis.TRANSLATION_TOLERANCE_METRES;
import static frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig.P;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Swerve.Chassis.VelocityPIDConfig;
import frc.robot.RunnymedeUtils;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public abstract class BaseDriveCommand extends LoggingCommand {
    protected final SwerveSubsystem swerve;

    public BaseDriveCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);

        SmartDashboard.putString("Drive/ToFieldPosition/current", "");
        SmartDashboard.putString("Drive/ToFieldPosition/delta", "");
        SmartDashboard.putString("Drive/ToFieldPosition/desired", "");
        SmartDashboard.putString("Drive/ToFieldPosition/velocity", "");
    }

    /**
     * Utility function to compute the required rotation speed of the robot given its current
     * heading. Uses a PID controller to compute the offset.
     *
     * @param target the desired heading of the robot
     * @return The required rotation speed of the robot
     * @see frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig
     */
    protected final Rotation2d computeOmega(Rotation2d target) {
        return computeOmega(target, swerve.getPose().getRotation());
    }

    /**
     * Utility function to compute the required rotation speed of the robot given the heading
     * provided. Uses a PID controller to compute the offset.
     *
     * @param target the desired heading of the robot
     * @param current the current heading of the robot
     * @return The required rotation speed of the robot
     * @see frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig
     */
    private static Rotation2d computeOmega(Rotation2d target, Rotation2d current) {

        double targetRadians  = target.getRadians();
        double currentRadians = current.getRadians();
        return computeOmegaForOffset(Rotation2d.fromRadians(targetRadians - currentRadians));

    }

    public static Rotation2d computeOmegaForOffset(Rotation2d offset) {

        double error = offset.getRadians();

        error = error % (2 * Math.PI);
        if (error > Math.PI) {
            error -= (2 * Math.PI);
        }
        else if (error < -Math.PI) {
            error += (2 * Math.PI);
        }

        final Rotation2d result;
        if (Math.abs(error) <= ROTATION_TOLERANCE.getRadians()) {
            // close enough - done!
            result = Rotation2d.fromRadians(0);
        }
        else {
            double omegaRadians = error * P * MAX_ROTATIONAL_VELOCITY_PER_SEC.getRadians();
            if (Math.abs(omegaRadians) < MIN_ROTATIONAL_VELOCITY_PER_SEC.getRadians()) {
                omegaRadians = Math.signum(omegaRadians) * MIN_ROTATIONAL_VELOCITY_PER_SEC.getRadians();
            }
            result = Rotation2d.fromRadians(omegaRadians);
        }

        return result;
    }


    /**
     * Return a velocity that will traverse the specified translation as fast as possible without
     * overshooting the location. The initial speed is expected to be 0 and the final speed is
     * expected to be 0.
     *
     * @param translationToTravel the desired translation to travel
     * @param maxSpeed the maximum speed to travel in Metres per Second
     * @return the velocity vector, in metres per second that the robot can safely travel
     * to traverse the distance specified
     */
    private static Translation2d computeVelocity(Translation2d translationToTravel, double maxSpeed) {

        double distanceMetres = translationToTravel.getNorm();
        double absDistMetres  = Math.abs(distanceMetres);

        // don't worry about tiny translations
        if (absDistMetres < TRANSLATION_TOLERANCE_METRES) {
            return new Translation2d();
        }

        // safety code
        if (maxSpeed > MAX_TRANSLATION_SPEED_MPS) {
            maxSpeed = MAX_TRANSLATION_SPEED_MPS;
        }
        else {
            // todo: remoe this option entirely
            maxSpeed = MAX_TRANSLATION_SPEED_MPS;
        }

        double xSign         = Math.signum(translationToTravel.getX());
        double ySign         = Math.signum(translationToTravel.getY());

        double decelDistance = Math.abs(DECEL_FROM_MAX_TO_STOP_DIST_METRES);

        // TODO: This logic below is incorrect. Delay this optimization until we figure it out.
        // double decelDistRatio = absDistMetres / DECEL_FROM_MAX_TO_STOP_DIST_METRES;
        // if (decelDistRatio < 1) {
        // decelDistance = decelDistance * decelDistRatio;
        // }


        final double speed;

        if (absDistMetres >= decelDistance) {
            // cruising
            speed = maxSpeed;
        }
        else {
            // decelerating
            double pctToGo = absDistMetres / decelDistance;
            speed = maxSpeed * pctToGo * VelocityPIDConfig.P;
        }


        Rotation2d angle = translationToTravel.getAngle();

        return new Translation2d(xSign * speed * Math.abs(angle.getCos()), ySign * speed * Math.abs(angle.getSin()));
    }

    /**
     * Drive as fast as safely possible to the specified pose.
     *
     * @param desiredPose the desired location on the field
     */
    protected final void driveToFieldPose(Pose2d desiredPose) {
        driveToFieldPose(desiredPose, MAX_TRANSLATION_SPEED_MPS);
    }

    /**
     * Drive as fast as safely possible to the specified pose, up ot the max speed specified.
     *
     * @param desiredPose the desired location on the field
     */
    protected final void driveToFieldPose(Pose2d desiredPose, double maxSpeedMPS) {
        Pose2d        current  = swerve.getPose();
        Transform2d   delta    = RunnymedeUtils.difference(desiredPose, current);

        Translation2d velocity = computeVelocity(delta.getTranslation(), maxSpeedMPS);
        Rotation2d    omega    = computeOmega(desiredPose.getRotation());

        // log("Current: " + format(current)
        // + " Delta: " + format(delta.getTranslation()) + " m @ " + format(delta.getRotation())
        // + " Target: " + format(desiredPose)
        // + " Velocity: " + format(velocity) + "m/s @ " + format(omega) + "/s");

        SmartDashboard.putString("Drive/ToFieldPosition/current", format(current));
        SmartDashboard.putString("Drive/ToFieldPosition/delta", format(delta.getTranslation()) + " m @ "
            + format(delta.getRotation()));
        SmartDashboard.putString("Drive/ToFieldPosition/desired", format(desiredPose));
        SmartDashboard.putString("Drive/ToFieldPosition/velocity", format(velocity) + "m/s @ " + format(omega) + "/s");

        swerve.driveFieldOriented(velocity, omega);
    }

    /**
     * Returns true when the robot is located within TRANSLATION_TOLERANCE_METRES of the desired
     * location
     */
    protected final boolean isCloseEnough(Translation2d desiredLocation) {
        Translation2d delta = desiredLocation.minus(swerve.getPose().getTranslation());
        return Math.abs(delta.getNorm()) <= TRANSLATION_TOLERANCE_METRES;
    }

    /**
     * Returns true when the robot heading is within ROTATION_TOLERANCE_RADIANS of the desired
     * location
     */
    protected final boolean isCloseEnough(Rotation2d desiredHeading) {
        Rotation2d delta = desiredHeading.minus(swerve.getPose().getRotation());
        return Math.abs(delta.getRadians()) <= ROTATION_TOLERANCE.getRadians();
    }

    /**
     * Compute the heading required to face the specified position on the field.
     * <p>
     * This handy utility enables the user to specify a field position, get
     * the required headiang for it, and pass it into computeOmega to
     * determine the required omega to face that position - even as the robot
     * moves across the field.
     * <code>
     *     Rotation2d heading getHeadingToFieldPosition(speaker)
     *     Rotation2d omega computeOmega(heading)
     * </code>
     * <p>
     * In other words, this utility enables the user to "lock on a target".
     *
     * @param target field position
     * @return the heading toward that position.
     * @see #computeOmega(Rotation2d)
     */
    protected final Rotation2d getHeadingToFieldPosition(Translation2d target) {
        Translation2d current = swerve.getPose().getTranslation();
        Translation2d delta   = target.minus(current);
        return delta.getAngle();
    }
}
