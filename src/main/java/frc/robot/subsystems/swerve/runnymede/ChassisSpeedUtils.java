package frc.robot.subsystems.swerve.runnymede;


import static frc.robot.Constants.Swerve.Chassis.MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;
import static frc.robot.Constants.Swerve.Chassis.ROTATION_TOLERANCE_RADIANS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ChassisSpeedUtils {

    private ChassisSpeedUtils() {}

    /**
     * @param vX metres per second
     * @param vY metres per second
     * @param omega radians per second
     */
    public static ChassisSpeeds fromRobotRelativeSpeeds(double vX, double vY, double omega) {
        return new ChassisSpeeds(vX, vY, omega);
    }

    /**
     * @param vX metres per second
     * @param vY metres per second
     * @param omega radians per second
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(double vX, double vY, double omega, Rotation2d robotHeading) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, omega, robotHeading);
    }

    /**
     * @param vX metres per second
     * @param vY metres per second
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(double vX, double vY, Rotation2d desiredHeading, Rotation2d robotHeading) {
        // TODO: this is a very crude calculation and can be greatly improved
        double currentHeadingRadians = robotHeading.getRadians();
        double desiredHeadingRadians = desiredHeading.getRadians();
        double error = desiredHeadingRadians - currentHeadingRadians;
        final double omega;
        if (Math.abs(error) > ROTATION_TOLERANCE_RADIANS) {
            if (error > .25) {
                omega = Math.signum(error) * MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;
            } else if (error > .15) {
                omega = Math.signum(error) * .5 * MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;
            } else {
                omega = Math.signum(error) * .2 * MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC;
            }
        } else {
            omega = 0;
        }
        return fromFieldRelativeSpeeds(vX, vY, omega, robotHeading);
    }
}
