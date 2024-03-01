package frc.robot.subsystems.swerve.runnymede;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;



public class SimulatedSwerveModule {
    private final Timer       timer = new Timer();
    private double            dt;
    private double            fakePos;
    private double            fakeSpeed;
    private double            lastTime;
    private SwerveModuleState state;

    public SimulatedSwerveModule() {
        this.timer.start();
        this.lastTime  = this.timer.get();
        this.state     = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
        this.fakeSpeed = 0.0;
        this.fakePos   = 0.0;
        this.dt        = 0.0;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.dt         = this.timer.get() - this.lastTime;
        this.lastTime   = this.timer.get();
        this.state      = desiredState;
        this.fakeSpeed  = desiredState.speedMetersPerSecond;
        this.fakePos   += this.fakeSpeed * this.dt;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.fakePos, this.state.angle);
    }

    public SwerveModuleState getState() {
        return this.state;
    }
}