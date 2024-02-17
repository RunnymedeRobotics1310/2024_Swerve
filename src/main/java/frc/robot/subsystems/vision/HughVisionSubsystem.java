package frc.robot.subsystems.vision;

import java.util.ArrayList;
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

    NetworkTableEntry                  tid                                  = table.getEntry("tid");

    NetworkTableEntry                  json                                 = table.getEntry("json");

    private BotTarget                  botTarget                            = null;

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

    public double getTargetAreaPercent() {
        return ta.getDouble(-1.0);
    }


    /**
     * Determine if a vision target of the current type is found.
     * <p>
     * Use {@link #setVisionTarget(BotTarget)} to set the vision target type
     */
    public boolean isVisionTargetFound() {
        return tv.getDouble(-1) == 1;
    }


    public boolean isVisionTargetClose() {
        // FIXME: finish this
        if (PIPELINE_APRIL_TAG_DETECT != pipeline.getInteger(-1)) {
            return false;
        }

        // FIXME: Check 10% - it's based on 2023 robot.
        double pct = getTargetAreaPercent();
        if (isVisionTargetFound() && pct > 10) {
            System.out.println("Vision target found and target area is " + pct + " which tells us we are close to the target");
            return true;
        }
        return false;

    }


    @Override
    public void periodic() {
        // read values periodically and post to smart dashboard periodically
        SmartDashboard.putBoolean("LimelightHugh/Target Found", isVisionTargetFound());
        SmartDashboard.putNumber("LimelightHugh/tx-value", tx.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightHugh/ty-value", ty.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightHugh/ta-value", ta.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightHugh/l-value", tl.getDouble(-1.0));
        SmartDashboard.putNumber("LimelightHugh/Cam Mode", camMode.getInteger(-1L));
        SmartDashboard.putNumber("LimelightHugh/LED mode", ledMode.getInteger(-1L));
        SmartDashboard.putNumber("LimelightHugh/Pipeline", pipeline.getInteger(-1L));
        SmartDashboard.putNumberArray("LimelightHugh/Botpose", getBotPose());
        SmartDashboard.putNumber("LimelightHugh/Number of Tags", getNumActiveTargets());
        SmartDashboard.putString("LimelightHugh/AprilTagInfo", aprilTagInfoArrayToString(getVisibleTagInfo()));
    }

    /**
     * Get the limelight coordinates for the target (i.e. with respect to the limelight origin, NOT
     * the robot!!)
     *
     * @return limelight target coordinates
     */
    private double[] getTarget() {
        double[] d = new double[2];
        d[0] = tx.getDouble(-1.0);
        d[1] = ty.getDouble(-1.0);
        return d;
    }

    /**
     * Get the limelight X angle measurement to the target.
     *
     * @return limelight X target coordinates
     */
    private double getTargetX() {
        return tx.getDouble(-1.0);
    }

    /**
     * Get the limelight Y angle measurement to the target.
     *
     * @return limelight Y target coordinates
     */
    private double getTargetY() {
        return ty.getDouble(-1.0);
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

        double   latency    = botPose[6];
        Pose2d   pose       = toPose2D(botPose);

        if (numTargets < 1) {
            return null;
        }

        // if bot is floating in mid air, return null
        if (botPose[2] > 1) {
            return null;
        }

        else {

            PoseConfidence poseConfidence = PoseConfidence.HIGH;
            if (numTargets == 1) {
                poseConfidence = PoseConfidence.MID;
            }
            return new VisionPositionInfo(pose, latency, poseConfidence);
        }

    }

    public double[] getBotPose() {
        // Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        // if (alliance.get().equals(DriverStation.Alliance.Red)) {
        // return botpose_wpired.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0, 0, 0 });
        // }
        // else
        return botpose_wpiblue.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0, 0, 0 });
    }

    // private static Pose3d toPose3D(double[] inData) {
    // if (inData.length < 6) {
    // System.err.println("Bad LL 3D Pose Data!");
    // return new Pose3d();
    // }
    // return new Pose3d(
    // new Translation3d(inData[0], inData[1], inData[2]),
    // new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
    // Units.degreesToRadians(inData[5])));
    // }

    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d    r2d    = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    private Pose2d getBotPose2d_wpiBlue() {

        double[] result = getBotPose();
        return toPose2D(result);
    }



    /**
     *
     * @since 2024-02-10
     */
    public Rotation2d getNoteOffset() {
        return null;
    }

    /**
     *
     * @since 2024-02-10
     */
    public BotTarget getVisionTarget() {
        return botTarget;
    }

    /**
     *
     * @since 2024-02-10
     */
    public void setVisionTarget(BotTarget botTarget) {

        this.botTarget = botTarget;

        switch (botTarget) {
        case BLUE_SPEAKER:
            activeAprilTagTargets = TARGET_BLUE_SPEAKER;
            break;

        case BLUE_AMP:
            activeAprilTagTargets = TARGET_BLUE_AMP;
            break;

        case BLUE_SOURCE:
            activeAprilTagTargets = TARGET_BLUE_SOURCE;
            break;

        case BLUE_STAGE:
            activeAprilTagTargets = TARGET_BLUE_STAGE;
            break;

        case RED_SPEAKER:
            activeAprilTagTargets = TARGET_RED_SPEAKER;
            break;

        case RED_AMP:
            activeAprilTagTargets = TARGET_RED_AMP;
            break;

        case RED_SOURCE:
            activeAprilTagTargets = TARGET_RED_SOURCE;
            break;

        case RED_STAGE:
            activeAprilTagTargets = TARGET_RED_STAGE;
            break;
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
        for (AprilTagInfo tag : visibleTags) {
            for (Integer targetTagId : activeAprilTagTargets) {
                if (targetTagId == tag.tid()) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     *
     * @since 2024-02-10
     */
    public boolean isAlignedWithTarget() {
        return false;
    }

    /**
     *
     * @since 2024-02-10
     */
    public double getDistanceToTargetMetres() {
        return -1;
    }

    /**
     *
     * @since 2024-02-10
     */
    public Rotation2d getTargetOffset() {
        return null;
    }

    /**
     *
     * @since 2024-02-10
     */
    public Translation2d getRobotTranslationToTarget() {
        return null;
    }
}