package ca.team1310.swervedrive.vision;

import ca.team1310.swervedrive.core.config.CoreSwerveConfig;
import ca.team1310.swervedrive.odometry.FieldAwareSwerveDrive;
import ca.team1310.swervedrive.telemetry.VisionAwareDriveTelemetry;
import edu.wpi.first.math.geometry.Pose2d;

import static ca.team1310.swervedrive.vision.PoseConfidence.NONE;

public class VisionAwareSwerveDrive extends FieldAwareSwerveDrive {

    private final HughVision hugh;

    public VisionAwareSwerveDrive(CoreSwerveConfig coreSwerveConfig, VisionConfig visionConfig) {
        super(coreSwerveConfig);
        this.hugh = new HughVision(visionConfig);
    }

    public void updateOdometry() {
        super.updateOdometry();

        VisionPositionInfo visPosInfo = getVisionPositionInfo();
        if (visPosInfo != null) {
            super.addVisionMeasurement(visPosInfo.pose(), visPosInfo.timestampSeconds(), visPosInfo.deviation());
        }
    }

    private VisionPositionInfo getVisionPositionInfo() {
        VisionPositionInfo visPosInfo;
        PoseConfidence     confidence;

        try {
            Pose2d odometryPose = super.getPose();
            visPosInfo = hugh.getVisionPositionInfo(odometryPose);
            confidence = visPosInfo.confidence();
        }
        catch (InvalidVisionDataException e) {
            visPosInfo = null;
            confidence = NONE;
        }

        return confidence == NONE ? null : visPosInfo;
    }

    public VisionAwareDriveTelemetry getVisionTelemetry() {
        return hugh.getVisionTelemetry();
    }
}