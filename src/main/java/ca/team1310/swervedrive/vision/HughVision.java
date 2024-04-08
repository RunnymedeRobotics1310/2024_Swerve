package ca.team1310.swervedrive.vision;

import ca.team1310.swervedrive.telemetry.VisionAwareDriveTelemetry;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Handles the April Tag Limelight
 */
class HughVision {

    NetworkTable                       table           = NetworkTableInstance.getDefault().getTable("limelight-hugh");
    // output
    private final NetworkTableEntry    tx              = table.getEntry("tx");
    private final NetworkTableEntry    ty              = table.getEntry("ty");
    private final NetworkTableEntry    ta              = table.getEntry("ta");
    private final NetworkTableEntry    tl              = table.getEntry("tl");
    private final NetworkTableEntry    botpose_wpiblue = table.getEntry("botpose_wpiblue");
    private final NetworkTableEntry    tid             = table.getEntry("tid");
    private final NetworkTableEntry    priorityId      = table.getEntry("priorityid");
    private final NetworkTable         poseTable       = NetworkTableInstance.getDefault().getTable("LimelightPose");
    private final DoubleArrayPublisher limelightPub    = poseTable.getDoubleArrayTopic("Robot").publish();
    private final double               fieldExtentMetresX;
    private final double               fieldExtentMetresY;
    private final double               maxAmbiguity;
    private final double               highQualityAmbiguity;
    private final double               maxVisposDeltaDistanceMetres;
    private PoseEstimate               poseEstimate;
    private VisionPositionInfo         visPosInfo;

    HughVision(VisionConfig cfg) {
        this.fieldExtentMetresX           = cfg.fieldExtentMetresX();
        this.fieldExtentMetresY           = cfg.fieldExtentMetresY();
        this.maxAmbiguity                 = cfg.maxAmbiguity();
        this.highQualityAmbiguity         = cfg.highQualityAmbiguity();
        this.maxVisposDeltaDistanceMetres = cfg.maxVisposeDeltaDistanceMetres();
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        pipeline.setNumber(cfg.pipelineAprilTagDetect());
        // inputs/configs
        NetworkTableEntry camMode = table.getEntry("camMode");
        camMode.setNumber(cfg.camModeVision());
    }

    /**
     * Compute the position information using limelight data and the current odometry pose.
     *
     * @param currentOdometryPose the current odometry pose
     * @return VisionPositionInfo.
     * @throws InvalidVisionDataException if the vision data is invalid
     */
    VisionPositionInfo getVisionPositionInfo(Pose2d currentOdometryPose) throws InvalidVisionDataException {
        this.poseEstimate = getBotPoseEstimate(botpose_wpiblue);
        this.visPosInfo   = calcVisionPositionInfo(poseEstimate, currentOdometryPose);
        return visPosInfo;
    }

    VisionAwareDriveTelemetry getVisionTelemetry() {
        publishToField(poseEstimate.pose);
        return new VisionAwareDriveTelemetry(
            visPosInfo.confidence() != PoseConfidence.NONE,
            visPosInfo.confidence(),
            priorityId.getDouble(-1),
            tid.getDouble(-1.0),
            tx.getDouble(-1.0),
            ty.getDouble(-1.0),
            ta.getDouble(-1.0),
            tl.getDouble(-1.0),
            poseEstimate.pose.getTranslation().toVector().getData(),
            poseEstimate.avgTagDist,
            poseEstimate.tagCount,
            aprilTagInfoArrayToString(poseEstimate.rawFiducials),
            visPosInfo.odometryDistDelta());
    }

    private String aprilTagInfoArrayToString(RawFiducial[] rawFiducials) {
        if (rawFiducials == null) {
            return "null";
        }
        StringBuilder sb = new StringBuilder();
        for (RawFiducial rawFiducial : rawFiducials) {
            sb.append("[id:").append(rawFiducial.id)
                .append(",distToRobot:").append(rawFiducial.distToRobot)
                .append(",ambiguity:").append(rawFiducial.ambiguity)
                .append(",txnc:").append(rawFiducial.txnc)
                .append(",tync:").append(rawFiducial.tync)
                .append(",ta:").append(rawFiducial.ta)
                .append("]");
        }
        return sb.toString();
    }

    private static Pose2d toPose2D(double[] inData) throws InvalidVisionDataException {
        if (inData.length < 6) {
            throw new InvalidVisionDataException("Bad LL 2D Pose Data!");
        }

        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        // Add 180deg to rotation because Hugh is on rear of bot
        Rotation2d    r2d    = Rotation2d.fromDegrees((inData[5] + 180) % 360);
        return new Pose2d(tran2d, r2d);
    }

    private static double extractBotPoseEntry(double[] inData, int position) throws InvalidVisionDataException {
        if (inData.length < position + 1) {
            throw new InvalidVisionDataException(
                "Cannot reference position: " + position + " in data array of length: " + inData.length);
        }
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(NetworkTableEntry botpose_wpiblue) throws InvalidVisionDataException {
        var           poseArray         = botpose_wpiblue.getDoubleArray(new double[0]);
        var           pose              = toPose2D(poseArray);
        double        latency           = extractBotPoseEntry(poseArray, 6);
        int           tagCount          = (int) extractBotPoseEntry(poseArray, 7);
        double        tagSpan           = extractBotPoseEntry(poseArray, 8);
        double        tagDist           = extractBotPoseEntry(poseArray, 9);
        double        tagArea           = extractBotPoseEntry(poseArray, 10);
        // getlastchange() in microseconds, ll latency in milliseconds
        var           timestampSeconds  = (botpose_wpiblue.getLastChange() / 1000000.0) - (latency / 1000.0);

        RawFiducial[] rawFiducials      = new RawFiducial[tagCount];
        int           valsPerFiducial   = 7;
        int           expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length == expectedTotalVals) {
            for (int i = 0; i < tagCount; i++) {
                int    baseIndex    = 11 + (i * valsPerFiducial);
                int    id           = (int) poseArray[baseIndex];
                double txnc         = poseArray[baseIndex + 1];
                double tync         = poseArray[baseIndex + 2];
                double ta           = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot  = poseArray[baseIndex + 5];
                double ambiguity    = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(pose, timestampSeconds, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials);
    }

    private void publishToField(Pose2d llPose) {
        // If you have a Field2D you can easily push it that way here.
        limelightPub.set(new double[] {
                llPose.getX(),
                llPose.getY(),
                llPose.getRotation().getDegrees()
        });
    }

    /**
     * Get the position of the robot as computed by the Vision Subsystem. Includes latency data.
     * <p>
     * If no valid position can be returned (due to bad or erratic data, blocked view, etc.),
     * returns null
     *
     * @return position info or null
     * @since 2024-02-10
     */
    private VisionPositionInfo calcVisionPositionInfo(PoseEstimate poseEstimate, Pose2d odometryPose) {
        double         stdDevRatio     = 1310;
        PoseConfidence poseConfidence  = PoseConfidence.NONE;
        double         compareDistance = poseEstimate.pose.getTranslation().getDistance(odometryPose.getTranslation());

        // If pose is 0,0 or no tags in view, we don't actually have data - return null
        if (poseEstimate.pose.getX() > 0
            && poseEstimate.pose.getY() > 0
            && poseEstimate.rawFiducials.length >= 1
            && poseEstimate.pose.getX() < fieldExtentMetresX
            && poseEstimate.pose.getY() < fieldExtentMetresY) {

            // Get the "best" tag - assuming the first one is the best - TBD TODO
            RawFiducial rawFiducial = poseEstimate.rawFiducials[0];

            if (rawFiducial.ambiguity < maxAmbiguity) {
                // If the ambiguity is very low, use the data as is (or when disabled, to allow for
                // bot repositioning
                if (rawFiducial.ambiguity < highQualityAmbiguity || DriverStation.isDisabled()) {
                    stdDevRatio    = .01;
                    poseConfidence = PoseConfidence.HIGH;
                }
                else {
                    // We need to be careful with this data set. If the location is too far off,
                    // don't use it. Otherwise, scale confidence by distance.
                    if (compareDistance < maxVisposDeltaDistanceMetres) {
                        stdDevRatio    = Math.pow(rawFiducial.distToRobot, 2) / 2;
                        poseConfidence = PoseConfidence.MEDIUM;
                    }
                }
            }
        }

        Matrix<N3, N1> deviation = VecBuilder.fill(stdDevRatio, stdDevRatio, 5 * stdDevRatio);
        return new VisionPositionInfo(poseEstimate.pose, poseEstimate.timestampSeconds, deviation, poseConfidence,
            compareDistance);
    }
}