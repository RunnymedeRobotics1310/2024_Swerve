package frc.robot.subsystems.swerve.runnymede;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Inspired by YAGSL SwerveAbsoluteEncoder and CANCoderSwerve. Designed to be used only
 * internally inside the SwerveModule object.
 */
class CanCoder {

    /**
     * The maximum amount of times the swerve encoder will attempt to configure itself if failures
     * occur.
     */
    private final int      maximumRetries = 5;

    private final CANcoder encoder;
    private final double   absoluteEncoderOffset;

    boolean                readingError   = false;

    CanCoder(int canId, double absoluteEncoderOffset, boolean inverted) {
        encoder = new CANcoder(canId);
        encoder.clearStickyFaults();

        CANcoderConfigurator cfg = encoder.getConfigurator();
        cfg.apply(new CANcoderConfiguration()); // resets to factory defaults
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();
        cfg.refresh(magnetSensorConfiguration);
        cfg.apply(magnetSensorConfiguration
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(
                inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive));

        // note, we aren't bothering to push this offset up into the encoder, we will just manage it
        // here in this class
        this.absoluteEncoderOffset = absoluteEncoderOffset;
    }

    int getDeviceId() {
        return encoder.getDeviceID();
    }

    /**
     * Get the absolute position of the encoder.
     * <p>
     * Sets readingError = true if there is a problem using the encoder.
     *
     * @return Absolute position in degrees from [0, 360).
     */
    double getAbsolutePositionInDegrees() {
        readingError = isNotHealthy();
        if (readingError) {
            return 0;
        }

        StatusSignal<Double> angle = encoder.getAbsolutePosition().refresh();

        int                  retryCount;
        for (retryCount = 0; retryCount < maximumRetries; retryCount++) {
            if (angle.getStatus() == StatusCode.OK) {
                break;
            }
            angle = angle.waitForUpdate(0.005);
        }
        if (angle.getStatus() != StatusCode.OK) {
            readingError = true;
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " reading was faulty after retrying "
                + maximumRetries + " times. " + angle.getStatus().getDescription(), false);
            return 0;
        }
        if (retryCount > 0) {
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " read successfully but required " + retryCount
                + " retries to get valid data.", false);
        }

        return (angle.getValue() * 360 - absoluteEncoderOffset + 360) % 360;
    }

    private boolean isNotHealthy() {
        MagnetHealthValue strength = encoder.getMagnetHealth().getValue();
        switch (strength) {
        case Magnet_Green:
            return false;
        case Magnet_Orange: {
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " magnetic field is less than ideal.", false);
            return false;
        }
        case Magnet_Invalid:
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " not usable - magnet INVALID.", false);
        case Magnet_Red:
            DriverStation.reportWarning("CANCoder " + encoder.getDeviceID() + " not usable - magnet health RED.", false);
        }
        return true;
    }

}
