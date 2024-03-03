package frc.robot.subsystems.swerve.runnymede;



import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AngleMotor extends SparkMaxNeoMotor {

    /**
     * Save the setpoint for telemetry
     */
    private double setpointDegrees;


    /**
     * Configure the SparkMAX and its integrated PIDF (PID + feed forward) control.
     *
     * @param canBusId bus id
     * @param cfg motor configuration
     * @{link https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control}
     */
    AngleMotor(int canBusId, Constants.Swerve.Motor cfg) {
        super(canBusId);
        // instantiate & configure motor
        this.motor.setInverted(cfg.inverted);
        configureSparkMax(() -> motor.enableVoltageCompensation(cfg.nominalVoltage));
        configureSparkMax(() -> motor.setSmartCurrentLimit(cfg.currentLimitAmps));
        configureSparkMax(() -> motor.setClosedLoopRampRate(cfg.rampRate));

        // configure integrated encoder
        final double turnConversionFactor = 1 / (cfg.gearRatio * 360);
        configureSparkMax(() -> encoder.setPositionConversionFactor(turnConversionFactor));
        // todo: check that 60 is correct!
        configureSparkMax(() -> encoder.setVelocityConversionFactor(turnConversionFactor / 60));

        // Configure feedback of the PID controller as teh integrated controller
        pid.setFeedbackDevice(encoder);
        configureSparkMax(() -> pid.setP(cfg.p, 0));
        configureSparkMax(() -> pid.setI(cfg.i, 0));
        configureSparkMax(() -> pid.setD(cfg.d, 0));
        configureSparkMax(() -> pid.setFF(cfg.ff, 0));
        configureSparkMax(() -> pid.setIZone(cfg.iz, 0));
        configureSparkMax(() -> pid.setOutputRange(-1, 1, 0));

        configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(true));
        configureSparkMax(() -> pid.setPositionPIDWrappingMinInput(0));
        configureSparkMax(() -> pid.setPositionPIDWrappingMaxInput(90));

        setMotorBrake(false);

        burnFlash();
    }

    Rotation2d getPosition() {
        return Rotation2d.fromDegrees((encoder.getPosition() + 360) % 360);
    }

    /**
     * Set the integrated encoder position
     *
     * @param position Integrated encoder position - degrees for turning motor
     */
    void setInternalEncoderPositionDegrees(double position) {
        if (encoder.getPosition() != position) {
            configureSparkMax(() -> encoder.setPosition(position));
        }
    }


    void setReferenceDegrees(double setpoint, double feedforward) {
        this.setpointDegrees = setpoint;
        configureSparkMax(() -> pid.setReference(setpoint, CANSparkBase.ControlType.kPosition, 0, feedforward));
    }

    void updateTelemetry() {
        super.updateTelemetry("swerve/1310/module/angle/debug/");
        SmartDashboard.putNumber("swerve/1310/module/angle/setpointDeg", setpointDegrees);
        SmartDashboard.putNumber("swerve/1310/module/angle/positionDeg", getPosition().getDegrees());
    }
}
