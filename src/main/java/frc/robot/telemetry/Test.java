package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.test.SystemTestCommand;

public class Test {
    Test() {
    }

    public boolean                 enabled;
    public SystemTestCommand.Motor selectedMotor;
    public double                  motorSpeed  = -1310.0;
    public double                  motor2Speed = -1310.0;
    public Rotation2d              angle       = null;


    void post() {
        SmartDashboard.putBoolean(Telemetry.PREFIX + "Test Mode/Enabled", enabled);
        if (enabled) {
            SmartDashboard.putString(Telemetry.PREFIX + "Test Mode/Motor", selectedMotor == null ? "" : selectedMotor.toString());
            SmartDashboard.putString(Telemetry.PREFIX + "Test Mode/Motor Speed", String.format("%.1f", motorSpeed * 100) + " %");
            SmartDashboard.putString(Telemetry.PREFIX + "Test Mode/Motor 2 Speed",
                String.format("%.1f", motor2Speed * 100) + " %");
            SmartDashboard.putString(Telemetry.PREFIX + "Test Mode/Angle",
                angle == null ? "" : String.format("%.3f", angle.getDegrees()) + " degrees");
        }
    }
}
