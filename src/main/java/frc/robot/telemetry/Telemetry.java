package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Telemetry {

    public static final String    PREFIX = "1310/";

    public static TelemetryConfig config = new TelemetryConfig();

    public static Drive           drive  = new Drive();
    public static Hugh            hugh   = new Hugh();
    public static Swerve          swerve = new Swerve();
    public static Test            test   = new Test();

    private Telemetry() {
    }

    public static void post() {

        config.post();

        if (config.drive()) {
            drive.post();
        }
        if (config.hugh()) {
            hugh.post();
        }
        if (config.swerve()) {
            swerve.post();
        }
        test.post();
    }

    static String format(Rotation2d angle) {
        return angle == null ? "" : String.format("%.2f", angle.getDegrees()) + " deg";
    }

    static String format(Translation2d translation) {
        return translation == null ? ""
            : String.format("%.2f", translation.getX()) + "," + String.format("%.2f", translation.getY());
    }

    static String format(Pose2d pose) {
        return pose == null ? ""
            : format(pose.getTranslation()) + " @ " + format(pose.getRotation());
    }
}