package frc.robot.geometry;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceTranslation3d {
    private final Translation3d blue;
    private final Translation3d red;

    AllianceTranslation3d(double blueX, double blueY, double blueZ, double redX, double redY, double redZ) {
        blue = new Translation3d(blueX, blueY, blueZ);
        red  = new Translation3d(redX, redY, redZ);
    }

    public Translation3d get() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? blue : red;
    }

    public String toString() {
        return get().toString();
    }
}
