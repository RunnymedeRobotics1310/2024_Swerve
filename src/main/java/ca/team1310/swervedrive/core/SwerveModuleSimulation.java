package ca.team1310.swervedrive.core;

import ca.team1310.swervedrive.core.config.ModuleConfig;
import ca.team1310.swervedrive.telemetry.ModuleTelemetry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

class SwerveModuleSimulation implements SwerveModule {

    private final String        name;
    private final Translation2d location;
    private final Timer         timer = new Timer();
    private double              dt;
    private double              fakePos;
    private double              fakeSpeed;
    private double              lastTime;
    private SwerveModuleState   currentState;
    private SwerveModuleState   desiredState;

    public SwerveModuleSimulation(ModuleConfig cfg) {
        this.name     = cfg.name();
        this.location = new Translation2d(cfg.xPositionMetres(), cfg.yPositionMetres());
        this.timer.start();
        this.lastTime     = this.timer.get();
        this.currentState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
        this.fakeSpeed    = 0.0;
        this.fakePos      = 0.0;
        this.dt           = 0.0;
    }

    @Override
    public String getName() {
        return this.name;
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.fakePos, this.currentState.angle);
    }

    @Override
    public SwerveModuleState getState() {
        return this.currentState;
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState  = desiredState;
        this.dt            = this.timer.get() - this.lastTime;
        this.lastTime      = this.timer.get();
        this.currentState  = desiredState;
        this.fakeSpeed     = desiredState.speedMetersPerSecond;
        this.fakePos      += this.fakeSpeed * this.dt;
    }

    @Override
    public ModuleTelemetry getModuleTelemetry() {
        return new ModuleTelemetry(
            name,
            location,
            desiredState.speedMetersPerSecond,
            desiredState.angle.getDegrees(),
            0,
            0,
            0
        /*
         * angleEncoder.getPosition(),
         * angleMotor.getPosition(),
         * driveMotor.getDistance()
         */);
    }
}
