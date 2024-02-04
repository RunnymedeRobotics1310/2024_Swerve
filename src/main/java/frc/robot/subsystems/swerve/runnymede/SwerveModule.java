package frc.robot.subsystems.swerve.runnymede;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private final DriveMotor driveMotor;
    private final AngleMotor angleMotor;
    private final CanCoder encoder;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     * TODO: periodically update neo encoder from absolute encoder
     * TODO: figure out how to handle brownouts.
     */
    public SwerveModule(Constants.Swerve.Module cfg, Constants.Swerve.Motor driveCfg, Constants.Swerve.Motor angleCfg) {
        driveMotor = new DriveMotor(cfg.driveCANID, driveCfg, cfg.wheelRadiusMetres);
        angleMotor = new AngleMotor(cfg.angleCANID, angleCfg);
        encoder = new CanCoder(cfg.encoderCANID, cfg.encoderAbsoluteOffsetDegrees, false);

        double angle = encoder.getAbsolutePositionInDegrees();
        angleMotor.setInternalEncoderPositionDegrees(angle);
        if (encoder.readingError) {
            throw new IllegalStateException("Absolute encoder " + cfg.encoderCANID + " could not be read.");
        }
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getDistanceMetres(), angleMotor.getPosition());
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        Rotation2d currentHeading = angleMotor.getPosition();

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, currentHeading);

        /*
           If the angle error is close to 0 degrees, we are aligned properly, so we can apply full power
           to drive wheels. If the angle error is close to 90 degrees, driving in any direction does not
           help. Used cosine function on the error to scale the desired speed. If cosine is < 0 then scale
           to zero so that we don't invert the drive for no reason.
         */
        Rotation2d steerError = desiredState.angle.minus(currentHeading);
        double cosineScalar = steerError.getCos();
        desiredState.speedMetersPerSecond *= (cosineScalar < 0 ? 0 : cosineScalar);
        driveMotor.setReferenceMetresPerSecond(desiredState.speedMetersPerSecond, 0);

        angleMotor.setReferenceDegrees(desiredState.angle.getDegrees(), 0);

    }
}