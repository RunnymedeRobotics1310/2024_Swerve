package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {

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
    private static final long                      PIPELINE_RETROREFLECTIVE_NOTE_DETECT = 0;
    private static final long                      PIPELINE_APRIL_TAG_DETECT            = 1;
    private static final long                      PIPELINE_VISUAL                      = 2;
    private static final long                      PIPELINE_NEURALNET_NOTE_DETECT       = 7;

    NetworkTable                                   table                                = NetworkTableInstance.getDefault()
        .getTable("limelight");

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

    private Constants.VisionConstants.VisionTarget currentVisionTarget                  = Constants.VisionConstants.VisionTarget.NONE;

    public VisionSubsystem() {
        setVisionTarget(VisionTarget.APRILTAGS);
    }

    public double getTargetAreaPercent() {
        return ta.getDouble(-1.0);
    }

    public VisionTarget getCurrentVisionTarget() {
        return currentVisionTarget;
    }

    public void setVisionTarget(VisionTarget visionTarget) {

        this.currentVisionTarget = visionTarget;

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
     * Determine if a vision target of the current type is found.
     * <p>
     * Use {@link #setVisionTarget(VisionTarget)} to set the vision target type
     */
    public boolean isVisionTargetFound() {
        return tv.getDouble(-1) == 1;
    }


    public double getTargetDistanceCm() {
        return -1.0; // fixme: calculate distance
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
            (currentVisionTarget == VisionTarget.NOTE) && isVisionTargetFound());
        SmartDashboard.putBoolean("Tag", currentVisionTarget == VisionTarget.APRILTAGS && isVisionTargetFound());
        SmartDashboard.putNumber("Limelight tx-value", tx.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight ty-value", ty.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight ta-value", ta.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight l-value", tl.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight Cam Mode", camMode.getInteger(-1L));
        SmartDashboard.putNumber("Limelight LED mode", ledMode.getInteger(-1L));
        SmartDashboard.putNumber("Limelight Pipeline", pipeline.getInteger(-1L));
        SmartDashboard.putBoolean("Note Target Acquired", isNoteTargetAcquired());
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
    public double getTargetX() {
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

    /**
     * Get the position of the robot as computed by the Vision Subsystem. Includes latency data.
     * 
     * If no valid position can be returned (due to bad or erratic data, blocked view, etc.),
     * returns null
     * 
     * @return position info or null
     */
    public VisionPositionInfo getPositionInfo() {
        return null;
    }


}