package frc.robot.subsystems.swerve.runnymede;


import com.revrobotics.CANSparkBase;

import frc.robot.Constants;

public class DriveMotor extends SparkMaxNeoMotor {

    /**
     * Configure the SparkMAX and its integrated PIDF (PID + feed forward) control.
     *
     * @param canBusId bus id
     * @param cfg motor configuration
     * @param wheelRadiusMetres the wheel radius in metres
     * @{link https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control}
     */
    DriveMotor(int canBusId, Constants.Swerve.Motor cfg, double wheelRadiusMetres) {
        super(canBusId);

        // instantiate & configure motor
        this.motor.setInverted(cfg.inverted);
        configureSparkMax(() -> motor.enableVoltageCompensation(cfg.nominalVoltage));
        configureSparkMax(() -> motor.setSmartCurrentLimit(cfg.currentLimitAmps));
        configureSparkMax(() -> motor.setClosedLoopRampRate(cfg.rampRate));

        // configure integrated encoder
        final double positionConversionfactor = (60 * Math.PI * 2 * wheelRadiusMetres) / cfg.gearRatio;
        configureSparkMax(() -> encoder.setPositionConversionFactor(positionConversionfactor));
        configureSparkMax(() -> encoder.setVelocityConversionFactor(positionConversionfactor / 60));


        pid.setFeedbackDevice(encoder); // Configure feedback of the PID controller as the
                                        // integrated encoder.
        configureSparkMax(() -> pid.setP(cfg.p, 0));
        configureSparkMax(() -> pid.setI(cfg.i, 0));
        configureSparkMax(() -> pid.setD(cfg.d, 0));
        configureSparkMax(() -> pid.setFF(cfg.ff, 0));
        configureSparkMax(() -> pid.setIZone(cfg.iz, 0));
        configureSparkMax(() -> pid.setOutputRange(-1, 1, 0));
        configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(false));
        setMotorBrake(true);

        burnFlash();
    }

    double getDistanceMetres() {
        return encoder.getPosition();
    }

    void setReferenceMetresPerSecond(double setpointMPS, double feedforward) {
        configureSparkMax(() -> pid.setReference(setpointMPS, CANSparkBase.ControlType.kVelocity, 0, feedforward));
    }

    double getVelocityMetresPerSecond() {
        return encoder.getVelocity();
    }
}
