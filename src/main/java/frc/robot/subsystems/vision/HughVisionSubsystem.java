package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BotTarget;

/**
 * Handles the April Tag Limelight On Shooter Side
 */
public class HughVisionSubsystem extends SubsystemBase {

    private static final long          LED_MODE_PIPELINE                    = 0;
    private static final long          LED_MODE_OFF                         = 1;
    @SuppressWarnings("unused")
    private static final long          LED_MODE_BLINK                       = 2;
    @SuppressWarnings("unused")
    private static final long          LED_MODE_ON                          = 3;

    private static final long          CAM_MODE_VISION                      = 0;
    private static final long          CAM_MODE_DRIVER                      = 1;

    // configure more pipelines here
    @SuppressWarnings("unused")
    private static final long          PIPELINE_RETROREFLECTIVE_NOTE_DETECT = 1;
    private static final long          PIPELINE_APRIL_TAG_DETECT            = 0;
    private static final long          PIPELINE_VISUAL                      = 2;


    private static final double        TARGET_ALIGNMENT_THRESHOLD           = 5;

    NetworkTable                       table                                = NetworkTableInstance.getDefault()
        .getTable("limelight-hugh");

    // inputs/configs
    NetworkTableEntry                  ledMode                              = table.getEntry("ledMode");
    NetworkTableEntry                  camMode                              = table.getEntry("camMode");
    NetworkTableEntry                  pipeline                             = table.getEntry("pipeline");

    // output
    NetworkTableEntry                  tv                                   = table.getEntry("tv");
    NetworkTableEntry                  tx                                   = table.getEntry("tx");
    NetworkTableEntry                  ty                                   = table.getEntry("ty");
    NetworkTableEntry                  ta                                   = table.getEntry("ta");

    NetworkTableEntry                  tl                                   = table.getEntry("tl");
    NetworkTableEntry                  cl                                   = table.getEntry("cl");

    NetworkTableEntry                  botpose_wpiblue                      = table.getEntry("botpose_wpiblue");
    NetworkTableEntry                  botpose_wpired                       = table.getEntry("botpose_wpibred");

    NetworkTableEntry                  targetpose_robotspace                = table.getEntry("targetpose_robotspace");

    NetworkTableEntry                  tid                                  = table.getEntry("tid");

    NetworkTableEntry                  json                                 = table.getEntry("json");

    private BotTarget                  botTarget                            = BotTarget.NONE;

    private static final List<Integer> TARGET_BLUE_SPEAKER                  = List.of(7, 8);
    private static final List<Integer> TARGET_BLUE_SOURCE                   = List.of(9, 10);
    private static final List<Integer> TARGET_BLUE_AMP                      = List.of(6);
    private static final List<Integer> TARGET_BLUE_STAGE                    = List.of(14, 15, 16);

    private static final List<Integer> TARGET_RED_SPEAKER                   = List.of(3, 4);
    private static final List<Integer> TARGET_RED_SOURCE                    = List.of(1, 2);
    private static final List<Integer> TARGET_RED_AMP                       = List.of(5);
    private static final List<Integer> TARGET_RED_STAGE                     = List.of(11, 12, 13);
    private static final List<Integer> TARGET_NONE                          = List.of();

    private List<Integer>              activeAprilTagTargets                = TARGET_NONE;


    public HughVisionSubsystem() {
        this.pipeline.setNumber(PIPELINE_APRIL_TAG_DETECT);
        this.camMode.setNumber(CAM_MODE_VISION);
        this.ledMode.setNumber(LED_MODE_PIPELINE);
    }

    @Override
    public void periodic() {
        // read values periodically and post to smart dashboard periodically
        SmartDashboard.putString("LimelightHugh/BotTarget", getBotTarget().toString());
        SmartDashboard.putBoolean("LimelightHugh/Target Found", isCurrentTargetVisible());
        SmartDashboard.putNumber("LimelightHugh/tid", tid.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightHugh/tx", tx.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightHugh/ty", ty.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightHugh/ta", ta.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightHugh/tl", tl.getDouble(-1.0));
        double[] bp = getBotPose();
        SmartDashboard.putNumberArray("LimelightHugh/Botpose", bp == null ? new double[0] : bp);
        SmartDashboard.putNumber("LimelightHugh/Number of Tags", getNumActiveTargets());
        SmartDashboard.putString("LimelightHugh/AprilTagInfo", aprilTagInfoArrayToString(getVisibleTagInfo()));
        SmartDashboard.putNumber("LimelightHugh/DistanceToTarget", getDistanceToTargetMetres());
        SmartDashboard.putBoolean("LimelightHugh/AlignedWithTarget", isAlignedWithTarget());
    }

    /**
     * Get the limelight coordinates for the target (i.e. with respect to the limelight origin, NOT
     * the robot!!)
     *
     * @return limelight target coordinates
     */
    private double[] getTarget() {
        double[] d = new double[2];
        d[0] = tx.getDouble(Double.MIN_VALUE);
        d[1] = ty.getDouble(Double.MIN_VALUE);
        return d;
    }

    /**
     * Get the limelight X angle measurement to the target.
     *
     * @return limelight X target coordinates
     */
    private double getTargetX() {
        return tx.getDouble(Double.MIN_VALUE);
    }

    /**
     * Get the limelight Y angle measurement to the target.
     *
     * @return limelight Y target coordinates
     */
    private double getTargetY() {
        return ty.getDouble(Double.MIN_VALUE);
    }


    private int countOccurrences(String str, String subStr) {
        int count     = 0;
        int fromIndex = 0;

        if (str == null) {
            return 0;
        }
        // Check if the substring is not empty to avoid infinite loop
        if (subStr == null || subStr.isEmpty()) {
            return 0;
        }

        while ((fromIndex = str.indexOf(subStr, fromIndex)) != -1) {
            count++;
            fromIndex += subStr.length(); // Move to the end of the current occurrence to find the
                                          // next one
        }

        return count;
    }

    /**
     * Performs a String based parsing of limelight's json blob in order to obtain information on
     * multiple targets when they are in view, since limelight only gives easy access to the
     * closest/largest one. Not using JSON parsers libs due to up to 2.5ms parsing time.
     *
     * @return An array of AprilTagInfo objects, each representing a visible target.
     */
    private AprilTagInfo[] getVisibleTagInfo() {
        String                  jsonStr = json.getString(null);
        ArrayList<AprilTagInfo> tags    = new ArrayList<AprilTagInfo>();

        if (jsonStr != null) {
            int index = 0;
            while (index != -1) {
                index = jsonStr.indexOf("\"fID\":", index);
                if (index == -1)
                    break; // No more fID found

                int    fIDStart = index + 6;                                 // Start of fID value
                int    fIDEnd   = jsonStr.indexOf(",", fIDStart);
                String fID      = jsonStr.substring(fIDStart, fIDEnd).trim();

                int    txIndex  = jsonStr.indexOf("\"tx\":", fIDEnd);
                int    txStart  = txIndex + 5;                               // Start of tx value
                int    txEnd    = jsonStr.indexOf(",", txStart);
                if (txEnd == -1) { // Check if tx is the last value before the object ends
                    txEnd = jsonStr.indexOf("}", txStart);
                }
                String       tx      = jsonStr.substring(txStart, txEnd).trim();

                int          tid     = Integer.parseInt(fID);
                double       targetx = Double.parseDouble(tx);
                AprilTagInfo ati     = new AprilTagInfo(tid, targetx);
                tags.add(ati);

                index = txEnd; // Move index to end of the current tx to find the next fID
            }
        }

        AprilTagInfo[] tagRet = new AprilTagInfo[tags.size()];
        return tags.toArray(tagRet);
    }

    private String aprilTagInfoArrayToString(AprilTagInfo[] tagArray) {
        if (tagArray == null || tagArray.length == 0) {
            return "[]";
        }

        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for (int i = 0; i < tagArray.length; i++) {
            sb.append(tagArray[i].toString());
            if (i < tagArray.length - 1) {
                sb.append(", ");
            }
        }
        sb.append("]");
        return sb.toString();
    }

    private int getNumActiveTargets() {
        String jsonStr = json.getString(null);
        return countOccurrences(jsonStr, "fID");
    }

    private double[] getBotPose() {
        double[] botpose = botpose_wpiblue.getDoubleArray(new double[] { Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE,
                Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE });
        if (botpose[0] == Double.MIN_VALUE) {
            return null;
        }

        return botpose;
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d    r2d    = new Rotation2d((Units.degreesToRadians(inData[5]) + 180) % 360);
        return new Pose2d(tran2d, r2d);
    }

    /**
     *
     * PUBLIC API FROM HERE DOWN
     *
     */

    /**
     * Get the position of the robot as computed by the Vision Subsystem. Includes latency data.
     * 
     * If no valid position can be returned (due to bad or erratic data, blocked view, etc.),
     * returns null
     * 
     * @return position info or null
     * @since 2024-02-10
     */
    public VisionPositionInfo getPositionInfo() {
        double[] botPose    = getBotPose();
        int      numTargets = getNumActiveTargets();

        if (botPose == null) {
            return null;
        }

        double latency = botPose[6];
        Pose2d pose    = toPose2D(botPose);

        if (numTargets < 1) {
            return null;
        }

        // if bot is floating in mid air, return null
        if (botPose[2] > 1) {
            return null;
        }

        PoseConfidence poseConfidence = PoseConfidence.HIGH;
        if (numTargets == 1) {
            poseConfidence = PoseConfidence.MID;
        }

        return new VisionPositionInfo(pose, latency, poseConfidence);
    }

    /**
     * Return the current BotTarget
     *
     * @return the current BotTarget
     */
    public BotTarget getBotTarget() {
        return botTarget;
    }

    /**
     * Sets the subsystem up to be ready to target a specific field element.
     *
     * @param botTarget the field element to target
     */
    public void setBotTarget(BotTarget botTarget) {

        this.botTarget = botTarget;

        switch (botTarget) {
        case BLUE_SPEAKER -> activeAprilTagTargets = TARGET_BLUE_SPEAKER;
        case BLUE_AMP -> activeAprilTagTargets = TARGET_BLUE_AMP;
        case BLUE_SOURCE -> activeAprilTagTargets = TARGET_BLUE_SOURCE;
        case BLUE_STAGE -> activeAprilTagTargets = TARGET_BLUE_STAGE;
        case RED_SPEAKER -> activeAprilTagTargets = TARGET_RED_SPEAKER;
        case RED_AMP -> activeAprilTagTargets = TARGET_RED_AMP;
        case RED_SOURCE -> activeAprilTagTargets = TARGET_RED_SOURCE;
        case RED_STAGE -> activeAprilTagTargets = TARGET_RED_STAGE;
        default -> throw new IllegalArgumentException(botTarget.toString());
        }
    }

    /**
     * If any April tag in the actively set bot target is visible, return true.
     *
     * @return true if any April tag in the actively set bot target is visible
     * @since 2024-02-10
     */
    public boolean isCurrentTargetVisible() {
        AprilTagInfo[] visibleTags = getVisibleTagInfo();
        return Arrays.stream(visibleTags).anyMatch(tag -> activeAprilTagTargets.contains(tag.tid()));
    }

    /**
     * Check if the robot is aligned to the target, within a certain threshold. Threshold is defined
     * in TARGET_ALIGNMENT_THRESHOLD constant.
     *
     * @return true if the robot is aligned to the target, and a target tag is visible.
     */
    public boolean isAlignedWithTarget() {
        Rotation2d targetOffset = getTargetOffset();
        return targetOffset != null
            && targetOffset.getDegrees() > 180 - TARGET_ALIGNMENT_THRESHOLD
            && targetOffset.getDegrees() < 180 + TARGET_ALIGNMENT_THRESHOLD;
    }

    /**
     * Obtain the straight line distance to target, if any of the target's tags are in sight.
     *
     * @return distance to target in meters, or Double.MIN_VALUE if no targets are visible.
     */
    public double getDistanceToTargetMetres() {
        Translation2d robotPosition = getRobotTranslationToTarget();
        if (robotPosition == null) {
            return Double.MIN_VALUE;
        }
        return Math.hypot(robotPosition.getX(), robotPosition.getY());
    }

    /**
     * Obtains the relative heading to the target, if any of the target's tags are in sight.
     *
     * @return Rotation2d with the angle to target from center of bot (0,0). null if no targets are
     * visible.
     */
    public Rotation2d getTargetOffset() {
        int    currentTagId  = (int) tid.getInteger(-1);
        double angleToTarget = getTargetX();

        if (!activeAprilTagTargets.contains(currentTagId) || angleToTarget == Double.MIN_VALUE) {
            return null;
        }

        // Hugh is on the rear of the bot, so add 180 to values given to translate to front
        return new Rotation2d(angleToTarget + 180);
    }

    /**
     * Obtains the x & y translation of the robot to the target, if any of the target's tags are in
     * sight.
     *
     * @return Translation2d with the x&y to target from center of bot (0,0). null if no targets are
     * visible.
     */
    public Translation2d getRobotTranslationToTarget() {
        int      currentTagId         = (int) tid.getInteger(-1);
        double[] targetPoseInBotSpace = targetpose_robotspace.getDoubleArray(new double[] { Double.MIN_VALUE, Double.MIN_VALUE,
                Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE });

        if (!activeAprilTagTargets.contains(currentTagId) || targetPoseInBotSpace[0] == Double.MIN_VALUE) {
            return null;
        }

        // Limelight is returning [0] = X = Height, [1] = Y = Left/Right, [2] = Z = Forward/Back.
        // We need to return X = Forward/Back, and Y = Left/Right
        // Because Hugh is on the back of the Bot, X needs to have its sign reversed.
        return new Translation2d(targetPoseInBotSpace[2] * -1, targetPoseInBotSpace[1]);
    }

    @Override
    public String toString() {
        return "Hugh Vision Subsystem";
    }
}