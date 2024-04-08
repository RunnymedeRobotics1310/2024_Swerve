package frc.robot.telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

import ca.team1310.swervedrive.vision.VisionPositionInfo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Core swerve drive telemetry. IMPORTANT - the swerve variables here need to be set for
 * visualization to work. The names of the variables are important and should not be changed.
 */
public class Swerve {
    /** The Field! */
    public Field2d                             field                 = new Field2d();
    /** The number of swerve modules */
    public int                                 moduleCount           = -1310;
    /** The number of swerve modules */
    public double[]                            wheelLocations;
    /**
     * An array of rotation and velocity values describing the measured state of each swerve module
     */
    public double[]                            measuredStates;
    /**
     * An array of rotation and velocity values describing the desired state of each swerve module
     */
    public double[]                            desiredStates;
    /** The robot's current rotation based on odometry or gyro readings */
    public double                              robotRotation         = -1310.0;
    /** The maximum achievable speed of the modules, used to adjust the size of the vectors. */
    public double                              maxSpeed              = -1310.0;
    /** The units of the module rotations and robot rotation */
    public String                              rotationUnit          = "degrees";
    /** The distance between the left and right modules. */
    public double                              sizeLeftRight         = -1310.0;
    /** The distance between the front and back modules. */
    public double                              sizeFrontBack         = -1310.0;
    /**
     * The direction the robot should be facing when the "Robot Rotation" is zero or blank. This
     * option is often useful to align with odometry data or match videos. 'up', 'right', 'down' or
     * 'left'
     */
    public String                              forwardDirection      = "up";
    /**
     * The maximum achievable angular velocity of the robot. This is used to visualize the angular
     * velocity from the chassis speeds properties.
     */
    public double                              maxAngularVelocity    = -1310.0;
    /**
     * The maximum achievable angular velocity of the robot. This is used to visualize the angular
     * velocity from the chassis speeds properties.
     */
    public double[]                            measuredChassisSpeeds = new double[3];
    /** Describes the desired forward, sideways and angular velocity of the robot. */
    public double[]                            desiredChassisSpeeds  = new double[3];

    /*
     * The following variables are Runnymede extensions over the standard swerve drive.
     */

    public double                              rawImuDegrees;
    public double                              adjustedImuDegrees;

    public static Map<String, ModuleTelemetry> modules               = new LinkedHashMap<>();

    public ModuleTelemetry getModule(String name) {
        if (!modules.containsKey(name)) {
            modules.put(name, new ModuleTelemetry());
        }
        return modules.get(name);
    }

    public static class ModuleTelemetry {
        ModuleTelemetry() {
        }

        public double     speedMetersPerSecond;
        public double     angleDegrees;
        public double     absoluteEncoderPositionDegrees;
        public Rotation2d angleMotorPosition;
        public double     driveMotorPosition;
    }

    public ChassisSpeeds      swerve_robot_chassis_speeds = null;
    public Translation2d      swerve_velocity_field       = null;
    public VisionPositionInfo swerve_vispose              = null;
    public Pose2d             swerve_pose                 = null;



    /** Upload data to smartdashboard */
    public void post() {
        // Common Swerve
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("swerve/moduleCount", moduleCount);
        SmartDashboard.putNumberArray("swerve/wheelLocations", wheelLocations == null ? new double[0] : wheelLocations);
        SmartDashboard.putNumberArray("swerve/measuredStates", measuredStates == null ? new double[0] : measuredStates);
        SmartDashboard.putNumberArray("swerve/desiredStates", desiredStates == null ? new double[0] : desiredStates);
        SmartDashboard.putNumber("swerve/robotRotation", robotRotation);
        SmartDashboard.putNumber("swerve/maxSpeed", maxSpeed);
        SmartDashboard.putString("swerve/rotationUnit", rotationUnit);
        SmartDashboard.putNumber("swerve/sizeLeftRight", sizeLeftRight);
        SmartDashboard.putNumber("swerve/sizeFrontBack", sizeFrontBack);
        SmartDashboard.putString("swerve/forwardDirection", forwardDirection);
        SmartDashboard.putNumber("swerve/maxAngularVelocity", maxAngularVelocity);
        SmartDashboard.putNumberArray("swerve/measuredChassisSpeeds", measuredChassisSpeeds);
        SmartDashboard.putNumberArray("swerve/desiredChassisSpeeds", desiredChassisSpeeds);

        // Runnymede swerve only - YAGSL sets its own values
        for (String name : modules.keySet()) {
            ModuleTelemetry module = modules.get(name);
            SmartDashboard.putNumber(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Speed Setpoint",
                module.speedMetersPerSecond);
            SmartDashboard.putNumber(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Angle Setpoint", module.angleDegrees);

            SmartDashboard.putNumber(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Absolute Encoder Position Degrees",
                module.absoluteEncoderPositionDegrees);
            SmartDashboard.putString(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Angle Position",
                module.angleMotorPosition.toString());
            SmartDashboard.putNumber(Telemetry.PREFIX + "Swerve/Module[" + name + "]/Drive Position",
                module.driveMotorPosition);
        }



        // Telemetry extensions

        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/imuDegreesRaw", String.format("%.1f", rawImuDegrees));
        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/imuDegreesAdjusted", String.format("%.1f", adjustedImuDegrees));


        double omega = swerve_robot_chassis_speeds == null ? 0
            : Rotation2d.fromRadians(swerve_robot_chassis_speeds.omegaRadiansPerSecond).getDegrees();
        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/velocity_robot",
            swerve_robot_chassis_speeds == null ? ""
                : String.format("%.1f (%.1f,%.1f) m/s %.1f deg/s",
                    Math.hypot(swerve_robot_chassis_speeds.vxMetersPerSecond, swerve_robot_chassis_speeds.vyMetersPerSecond),
                    swerve_robot_chassis_speeds.vxMetersPerSecond, swerve_robot_chassis_speeds.vyMetersPerSecond, omega));

        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/velocity_field",
            swerve_velocity_field == null ? ""
                : String.format("%.1f (%.1f,%.1f) m/s %.1f deg/s",
                    swerve_velocity_field.getNorm(), swerve_velocity_field.getX(), swerve_velocity_field.getY(), omega));

        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/pose_vis", swerve_vispose == null ? "" : swerve_vispose.toString());

        SmartDashboard.putString(Telemetry.PREFIX + "Swerve/pose_odo", swerve_pose == null ? ""
            : String.format("(%.2f,%.2f) m %.1f deg", swerve_pose.getTranslation().getX(), swerve_pose.getTranslation().getY(),
                swerve_pose.getRotation().getDegrees()));

    }
}
