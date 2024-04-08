package ca.team1310.swervedrive.core;

import ca.team1310.swervedrive.core.config.ModuleConfig;
import ca.team1310.swervedrive.core.hardware.cancoder.CanCoder;
import ca.team1310.swervedrive.core.hardware.neosparkmax.NSMAngleMotor;
import ca.team1310.swervedrive.core.hardware.neosparkmax.NSMDriveMotor;
import ca.team1310.swervedrive.telemetry.ModuleTelemetry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final String               name;
    private final Translation2d        location;
    private final DriveMotor           driveMotor;
    private final AngleMotor           angleMotor;
    private final AbsoluteAngleEncoder angleEncoder;
    private final int                  internalEncoderUpdateFrequency;
    private int                        internalEncoderUpdateCount = 0;
    private SwerveModuleState          desiredState;


    SwerveModule(ModuleConfig cfg) {
        this.name                           = cfg.name();
        this.location                       = new Translation2d(cfg.xPositionMetres(), cfg.yPositionMetres());
        this.driveMotor                     = new NSMDriveMotor(cfg.driveMotorCanId(), cfg.driveMotorConfig(),
            cfg.wheelRadiusMetres());
        this.angleMotor                     = new NSMAngleMotor(cfg.angleMotorCanId(), cfg.angleMotorConfig());
        this.angleEncoder                   = new CanCoder(cfg.angleEncoderCanId(), cfg.angleEncoderAbsoluteOffsetDegrees(),
            cfg.absoluteAngleEncoderConfig());
        this.internalEncoderUpdateFrequency = cfg.angleMotorEncoderUpdateFrequency();
    }

    public String getName() {
        return name;
    }

    public Translation2d getLocation() {
        return location;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getDistance(), Rotation2d.fromDegrees(angleMotor.getPosition()));
    }

    SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(), Rotation2d.fromDegrees(angleMotor.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
        updateMotors();
        updateInternalEncoder();
    }

    private void updateMotors() {

        double     currentHeadingDegrees = angleMotor.getPosition();
        Rotation2d currentHeading        = Rotation2d.fromDegrees(currentHeadingDegrees);

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, currentHeading);

        /*
         * If the angle error is close to 0 degrees, we are aligned properly, so we can apply
         * full power to drive wheels. If the angle error is close to 90 degrees, driving in
         * any direction does not help. Used cosine function on the error to scale the
         * desired speed. If cosine is < 0 then scale to zero so that we don't invert the
         * drive for no reason.
         */
        Rotation2d steerError   = desiredState.angle.minus(currentHeading);
        double     cosineScalar = steerError.getCos();
        desiredState.speedMetersPerSecond *= (cosineScalar < 0 ? 0 : cosineScalar);
        driveMotor.setReferenceVelocity(desiredState.speedMetersPerSecond);

        angleMotor.setReferenceAngle(desiredState.angle.getDegrees());
    }

    private void updateInternalEncoder() {
        if (internalEncoderUpdateCount++ >= internalEncoderUpdateFrequency) {
            internalEncoderUpdateCount = 0;
            double angle = angleEncoder.getPosition();
            if (angle >= 0) {
                angleMotor.setEncoderPosition(angle);
            }
        }
    }

    ModuleTelemetry getModuleTelemetry() {
        return new ModuleTelemetry(
            name,
            location,
            desiredState.speedMetersPerSecond,
            desiredState.angle.getDegrees(),
            angleEncoder.getPosition(),
            angleMotor.getPosition(),
            driveMotor.getDistance());
    }
}
