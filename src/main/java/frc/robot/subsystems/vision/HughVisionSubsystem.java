package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.VisionTarget;

/**
 * Handles the April Tag Limelight On Shooter Side
 */
public class HughVisionSubsystem extends SubsystemBase {

    private static final long                      LED_MODE_PIPELINE                    = 0;
    private static final long                      LED_MODE_OFF                         = 1;
    @SuppressWarnings("unused")
    private static final long                      LED_MODE_BLINK                       = 2;
    @SuppressWarnings("unused")
    private static final long                      LED_MODE_ON                          = 3;

    private static final long                      CAM_MODE_VISION                      = 0;
    private static final long                      CAM_MODE_DRIVER                      = 1;

    // configure more pipelines here
    @SuppressWarnings("unused")
    private static final long                      PIPELINE_RETROREFLECTIVE_NOTE_DETECT = 1;
    private static final long                      PIPELINE_APRIL_TAG_DETECT            = 0;
    private static final long                      PIPELINE_VISUAL                      = 2;
    private static final long                      PIPELINE_NEURALNET_NOTE_DETECT       = 7;

    NetworkTable                                   table                                = NetworkTableInstance.getDefault()
        .getTable("limelight-hugh");

    // inputs/configs
    NetworkTableEntry                              ledMode                              = table.getEntry("ledMode");
    NetworkTableEntry                              camMode                              = table.getEntry("camMode");
    NetworkTableEntry                              pipeline                             = table.getEntry("pipeline");

    // output
    NetworkTableEntry                              tv                                   = table.getEntry("tv");
    NetworkTableEntry                              tx                                   = table.getEntry("tx");
    NetworkTableEntry                              ty                                   = table.getEntry("ty");
    NetworkTableEntry                              ta                                   = table.getEntry("ta");

    NetworkTableEntry                              tl                                   = table.getEntry("tl");
    NetworkTableEntry                              cl                                   = table.getEntry("cl");

    NetworkTableEntry                              botpose_wpiblue                      = table.getEntry("botpose_wpiblue");
    NetworkTableEntry                              botpose_wpired                       = table.getEntry("botpose_wpibred");

    NetworkTableEntry                              tid                                  = table.getEntry("tid");

    NetworkTableEntry                              json                                 = table.getEntry("json");

    private Constants.VisionConstants.VisionTarget visionTarget                         = Constants.VisionConstants.VisionTarget.NONE;

    public HughVisionSubsystem() {
        setVisionTarget(VisionTarget.APRILTAGS);
    }

    public double getTargetAreaPercent() {
        return ta.getDouble(-1.0);
    }


    /**
     * Determine if a vision target of the current type is found.
     * <p>
     * Use {@link #setVisionTarget(VisionTarget)} to set the vision target type
     */
    public boolean isVisionTargetFound() {
        return tv.getDouble(-1) == 1;
    }


    public boolean isNoteTargetAcquired() {
        // FIXME: finish this
        if (PIPELINE_NEURALNET_NOTE_DETECT != pipeline.getInteger(-1)) {
            return false;
        }

        // Check that a target it acquired.
        if (!isVisionTargetFound()) {
            return false;
        }

        // ***NOTE***: This was from 2023 with retroflective model - likely handling false
        // positives. Try without first.
        // is the target area larger than minPercentForConeAcquisition of the screen?
        // long minPercentForNoteAcquisition = 6;
        // if (getTargetAreaPercent() < minPercentForNoteAcquisition) {
        // return false;
        // }

        double[] tgt = getTarget();
        if (tgt[0] < 0 || tgt[1] < 0)
            return false;

        // FIXME: more checks
        return true;
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
        SmartDashboard.putBoolean("Limelight Target Found", isVisionTargetFound());
        SmartDashboard.putBoolean("Note",
            (visionTarget == VisionTarget.NOTE) && isVisionTargetFound());
        SmartDashboard.putBoolean("Tag", visionTarget == VisionTarget.APRILTAGS && isVisionTargetFound());
        SmartDashboard.putNumber("Limelight tx-value", tx.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight ty-value", ty.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight ta-value", ta.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight l-value", tl.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight Cam Mode", camMode.getInteger(-1L));
        SmartDashboard.putNumber("Limelight LED mode", ledMode.getInteger(-1L));
        SmartDashboard.putNumber("Limelight Pipeline", pipeline.getInteger(-1L));
        SmartDashboard.putBoolean("Note Target Acquired", isNoteTargetAcquired());
        SmartDashboard.putNumberArray("Botpose", getBotPose());
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
    /*
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
        String   jsonStr    = json.getString(null);
        double   latency    = botPose[6];

        int      numTargets = countOccurrences(jsonStr, "fID");
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
    public VisionTarget getVisionTarget() {
        return visionTarget;
    }

    /**
     *
     * @since 2024-02-10
     */
    public void setVisionTarget(VisionTarget visionTarget) {

        this.visionTarget = visionTarget;

        switch (visionTarget) {
        case APRILTAGS:
            this.pipeline.setNumber(PIPELINE_APRIL_TAG_DETECT);
            this.camMode.setNumber(CAM_MODE_VISION);
            this.ledMode.setNumber(LED_MODE_PIPELINE);
            break;
        case NOTE:
            this.pipeline.setNumber(PIPELINE_NEURALNET_NOTE_DETECT);
            this.camMode.setNumber(CAM_MODE_VISION);
            this.ledMode.setNumber(LED_MODE_PIPELINE);
            break;
        case NONE:
        default:
            this.pipeline.setNumber(PIPELINE_VISUAL);
            this.camMode.setNumber(CAM_MODE_DRIVER);
            this.ledMode.setInteger(LED_MODE_OFF);
            break;
        }
    }


    /**
     *
     * @since 2024-02-10
     */
    public boolean isCurrentTargetVisible() {
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