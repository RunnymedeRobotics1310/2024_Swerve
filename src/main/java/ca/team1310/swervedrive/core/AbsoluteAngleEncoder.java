package ca.team1310.swervedrive.core;

public interface AbsoluteAngleEncoder {

    /**
     * Get the absolute position of the encoder.
     * 
     * @return The absolute position of the encoder in degrees, from 0 to 360. Returns -1 on error.
     */
    double getPosition();
}
