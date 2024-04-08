package ca.team1310.swervedrive.core;

import ca.team1310.swervedrive.telemetry.ModuleTelemetry;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {

    String getName();

    Translation2d getLocation();

    SwerveModulePosition getPosition();

    SwerveModuleState getState();

    void setDesiredState(SwerveModuleState desiredState);

    ModuleTelemetry getModuleTelemetry();
}
