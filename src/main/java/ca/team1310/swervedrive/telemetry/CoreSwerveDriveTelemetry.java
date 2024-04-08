package ca.team1310.swervedrive.telemetry;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record CoreSwerveDriveTelemetry(
    double maxModuleSpeedMPS,
    double maxTranslationSpeedMPS,
    double maxRotationalVelocityRadPS,
    double trackWidthMetres,
    double wheelBaseMetres,
    double wheelRadiusMetres,
    ModuleTelemetry frontLeftModuleState,
    ModuleTelemetry frontRightModuleState,
    ModuleTelemetry backLeftModuleState,
    ModuleTelemetry backRightModuleState,
    ChassisSpeeds desiredChassisSpeeds,
    ChassisSpeeds measuredChassisSpeeds) {
}
