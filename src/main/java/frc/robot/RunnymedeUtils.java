package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class RunnymedeUtils {

    public static DriverStation.Alliance getRunnymedeAlliance() {
//        return DriverStation.Alliance.Blue;
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    }
}
