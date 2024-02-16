package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.robot.Constants.Swerve.Chassis.*;
import static frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig.P;

public abstract class BaseDriveCommand extends LoggingCommand {
    protected final SwerveSubsystem swerve;


    public BaseDriveCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    /**
     * Utility function to compute the required rotation speed of the robot given its current
     * heading. Uses a PID controller to compute the offset.
     *
     * @param heading the desired heading of the robot
     * @return The required rotation speed of the robot
     * @see frc.robot.Constants.Swerve.Chassis.HeadingPIDConfig
     */
    public final Rotation2d computeOmega(Rotation2d heading) {

        double error = (heading.getRadians() - swerve.getPose().getRotation().getRadians()) % (2 * Math.PI);
        if (error > Math.PI) {
            error -= (2 * Math.PI);
        }
        double omega = (error * P) * MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;

        return Rotation2d.fromRadians(omega);
    }

    /**
     * Return a velocity that will traverse the specified translation as fast as possible without
     * overshooting the location. The initial speed is expected to be 0 and the final speed is
     * expected to be 0.
     *
     * @param translationToTravel the desired translation to travel
     * @return the velocity vector, in metres per second that the robot can safely travel
     * to traverse the distance specified
     */
    public static Translation2d computeVelocity(Translation2d translationToTravel) {

        double distanceMetres = translationToTravel.getNorm();
        double sign           = Math.signum(distanceMetres);
        double absDistMetres  = Math.abs(distanceMetres);

        double maxSpeed       = MAX_TRANSLATION_SPEED_MPS;
        double decelDistance  = DECEL_FROM_MAX_TO_STOP_DIST_METRES;

        double decelDistRatio = absDistMetres / DECEL_FROM_MAX_TO_STOP_DIST_METRES;
        if (decelDistRatio < 1) {
            maxSpeed      = maxSpeed * decelDistRatio;
            decelDistance = decelDistance * decelDistRatio;
        }


        final double speed;

        if (absDistMetres >= decelDistance) {
            // cruising
            speed = sign * maxSpeed;
        }
        else {
            // decelerating
            double pctToGo = absDistMetres / decelDistance;
            speed = sign * maxSpeed * pctToGo;
        }

        Rotation2d angle = translationToTravel.getAngle();
        return new Translation2d(speed * angle.getCos(), speed * angle.getSin());
    }
}
