package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelemetryConfig {

    public SendableChooser<Boolean> telemetryDriveChooser  = new SendableChooser<>();
    public SendableChooser<Boolean> telemetryHughChooser   = new SendableChooser<>();
    public SendableChooser<Boolean> telemetrySwerveChooser = new SendableChooser<>();

    TelemetryConfig() {
        telemetryDriveChooser.setDefaultOption("Enabled", true);
        telemetryDriveChooser.addOption("Disabled", false);

        telemetryHughChooser.addOption("Enabled", true);
        telemetryHughChooser.setDefaultOption("Disabled", false);

        telemetrySwerveChooser.addOption("Enabled", true);
        telemetrySwerveChooser.setDefaultOption("Disabled", false);
    }

    void post() {
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Drive", telemetryDriveChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Hugh", telemetryHughChooser);
        SmartDashboard.putData(Telemetry.PREFIX + "Telemetry/Swerve", telemetrySwerveChooser);
    }


    public boolean drive() {
        Boolean enabled = telemetryDriveChooser.getSelected();
        return enabled != null && enabled;
    }

    public boolean hugh() {
        Boolean enabled = telemetryHughChooser.getSelected();
        return enabled != null && enabled;
    }

    public boolean swerve() {
        Boolean enabled = telemetrySwerveChooser.getSelected();
        return enabled != null && enabled;
    }
}
