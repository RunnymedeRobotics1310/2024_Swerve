package frc.robot.telemetry;

import static frc.robot.RunnymedeUtils.getRunnymedeAlliance;
import static frc.robot.telemetry.Telemetry.format;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive {
    Drive() {
    }

    public double        teleop_correctedHeadingDeg  = -1310.0;
    public double        teleop_vX                   = -1310.0;
    public double        teleop_vY                   = -1310.0;
    public double        teleop_ccwRotAngularVelPct  = -1310.0;
    public double        teleop_rawDesiredHeadingDeg = -1310.0;
    public double        teleop_boostFactor          = -1310.0;
    public String        teleop_mode                 = null;
    public boolean       teleop_lockOnSpeaker        = false;
    public Translation2d teleop_velocity             = null;
    public Rotation2d    teleop_theta                = null;
    public Rotation2d    teleop_omega                = null;
    public Transform2d   drive_to_pose_delta         = null;
    public Pose2d        drive_to_pose_desired       = null;
    public Translation2d drive_to_pose_velocity      = null;
    public Rotation2d    drive_to_pose_omega         = null;

    void post() {

        // Teleop
        SmartDashboard.putString(Telemetry.PREFIX + "Drive/Teleop/Alliance", getRunnymedeAlliance().name());

        SmartDashboard.putNumber(Telemetry.PREFIX + "Drive/Teleop/vX", teleop_vX);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Drive/Teleop/vY", teleop_vY);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Drive/Teleop/correctedHeadingDeg", teleop_correctedHeadingDeg);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Drive/Teleop/ccwRotAngularVelPct", teleop_ccwRotAngularVelPct);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Drive/Teleop/rawDesiredHeadingDeg", teleop_rawDesiredHeadingDeg);
        SmartDashboard.putNumber(Telemetry.PREFIX + "Drive/Teleop/boostFactor", teleop_boostFactor);
        SmartDashboard.putString(Telemetry.PREFIX + "Drive/Teleop/mode", teleop_mode == null ? "" : teleop_mode);
        SmartDashboard.putBoolean(Telemetry.PREFIX + "Drive/Teleop/lockOnSpeaker", teleop_lockOnSpeaker);

        SmartDashboard.putString(Telemetry.PREFIX + "Drive/Teleop/velocity",
            teleop_velocity == null ? ""
                : String.format("%.2f", teleop_velocity.getNorm()) + "m/s at "
                    + String.format("%.2f", teleop_velocity.getAngle().getDegrees()) + "deg");

        SmartDashboard.putString(Telemetry.PREFIX + "Drive/Teleop/theta ",
            teleop_theta == null ? "" : String.format("%.2f", teleop_theta.getDegrees()) + "deg");

        SmartDashboard.putString(Telemetry.PREFIX + "Drive/Teleop/omega",
            teleop_omega == null ? "" : String.format("%.2f", teleop_omega.getDegrees()) + "deg/s");


        // drive to position
        SmartDashboard.putString(Telemetry.PREFIX + "Drive/ToFieldPosition/delta",
            drive_to_pose_delta == null ? ""
                : format(drive_to_pose_delta.getTranslation()) + " m @ "
                    + format(drive_to_pose_delta.getRotation()));

        SmartDashboard.putString(Telemetry.PREFIX + "Drive/ToFieldPosition/desired",
            format(drive_to_pose_desired));

        SmartDashboard.putString(Telemetry.PREFIX + "Drive/ToFieldPosition/velocity",
            format(drive_to_pose_velocity) + "m/s @ " + format(drive_to_pose_omega) + "/s");

    }
}
