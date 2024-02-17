package frc.robot.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceTranslation2d {
    private final Translation2d blue;
    private final Translation2d red;

    AllianceTranslation2d(double blueX, double blueY, double redX, double redY) {
        blue = new Translation2d(blueX, blueY);
        red  = new Translation2d(redX, redY);
    }

    public Translation2d get() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? blue : red;
    }

    public String toString() {
        return get().toString();
    }
}
