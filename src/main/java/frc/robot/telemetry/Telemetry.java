package frc.robot.telemetry;

public class Telemetry {

    public static final String PREFIX = "1310/";

    public static Test         test   = new Test();

    private Telemetry() {
    }

    public static void post() {
        test.post();
    }
}