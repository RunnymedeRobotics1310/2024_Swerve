package ca.team1310.swervedrive;

import ca.team1310.swervedrive.telemetry.CoreSwerveDriveTelemetry;
import ca.team1310.swervedrive.telemetry.FieldAwareDriveTelemetry;
import ca.team1310.swervedrive.telemetry.VisionAwareDriveTelemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface RunnymedeSwerveDrive {

    /**
     * The main internal method for controlling the drivebase. This code does not apply any
     * limiters or validation, and should be used by implementing swerve drive subsystems
     * only.
     * <p>
     * Takes the desired chassis speeds of the robot - in a robot-oriented configuration.
     *
     * @param rawDesiredRobotOrientedVelocity The intended velocity of the robot chassis relative to
     * itself.
     */
    void drive(ChassisSpeeds rawDesiredRobotOrientedVelocity);

    boolean lock();

    void setModuleStateForTestMode(String moduleName, SwerveModuleState desiredState);


    // odometry-related
    // todo: figure out if this can be simplified or cleaned up
    // Do we need zeroGyro()? Should we rename the Gyro methods?

    void updateOdometry();

    void resetOdometry(Pose2d pose);

    Pose2d getPose();

    void zeroGyro();

    Rotation3d getGyroRotation3d();


    // telemetry-related
    // todo: consolidate these & consider performance implications.
    CoreSwerveDriveTelemetry getCoreTelemetry();

    FieldAwareDriveTelemetry getFieldTelemetryState();

    VisionAwareDriveTelemetry getVisionTelemetry();
}
