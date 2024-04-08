package ca.team1310.swervedrive.odometry.hardware;

import ca.team1310.swervedrive.odometry.Gyro;
import ca.team1310.swervedrive.telemetry.GyroTelemetry;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

public class MXPNavX implements Gyro {
    private final AHRS navx;
    private double     rollOffset;
    private double     pitchOffset;
    private double     yawOffset;

    public MXPNavX() {
        this.navx        = new AHRS(SerialPort.Port.kMXP);
        this.rollOffset  = navx.getRoll();
        this.pitchOffset = navx.getPitch();
        this.yawOffset   = navx.getYaw();
    }

    @Override
    public void zeroGyro() {
        rollOffset  = navx.getRoll();
        pitchOffset = navx.getPitch();
        yawOffset   = navx.getYaw();
    }

    @Override
    public double getRoll() {
        return navx.getRoll() - rollOffset;
    }

    @Override
    public double getPitch() {
        return navx.getPitch() - pitchOffset;
    }

    @Override
    public double getYaw() {
        return navx.getYaw() - yawOffset;
    }

    @Override
    public GyroTelemetry getTelemetryState() {
        return new GyroTelemetry(navx.getYaw(), this.getYaw());
    }
}
