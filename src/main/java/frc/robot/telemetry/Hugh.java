package frc.robot.telemetry;

import java.util.Arrays;

import ca.team1310.swervedrive.vision.PoseConfidence;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hugh {
    Hugh() {
    }

    public double         priorityId     = -1310.0;
    public double         tid            = -1310.0;
    public double         tx             = -1310.0;
    public double         ty             = -1310.0;
    public double         ta             = -1310.0;
    public double         tl             = -1310.0;
    public double[]       botpose        = null;
    public double         targetAvgDist  = -1310.0;
    public int            numTags        = -1310;
    public String         aprilTagInfo   = "null";
    public boolean        poseUpdate     = false;
    public PoseConfidence poseConfidence = PoseConfidence.NONE;
    public double         poseSwerveDiff = -1310.0;

    void post() {
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/PriorityId", "" + priorityId);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/tid", tid);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/tx", tx);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/ty", ty);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/ta", ta);
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/tl", tl);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/Botpose", Arrays.toString(botpose));
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/TargetAvgDist", targetAvgDist);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/NumTags", "" + numTags);
        SmartDashboard.putBoolean(Telemetry.PREFIX + "VisionHugh/PoseUpdate", poseUpdate);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/PoseConf", poseConfidence.name());
        SmartDashboard.putNumber(Telemetry.PREFIX + "VisionHugh/PoseSwerveDiff", poseSwerveDiff);
        SmartDashboard.putString(Telemetry.PREFIX + "VisionHugh/AprilTagInfo", aprilTagInfo);
    }
}
