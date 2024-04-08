package ca.team1310.swervedrive.telemetry;

import ca.team1310.swervedrive.vision.PoseConfidence;

public record VisionAwareDriveTelemetry(
    boolean poseUpdate,
    PoseConfidence poseConfidence,
    double priorityId,
    double tid,
    double tx,
    double ty,
    double ta,
    double tl,
    double[] botpose,
    double targetAvgDist,
    int numTags,
    String aprilTagInfo,
    double poseSwerveDiff) {
}