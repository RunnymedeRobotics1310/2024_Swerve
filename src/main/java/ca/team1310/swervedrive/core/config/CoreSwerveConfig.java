package ca.team1310.swervedrive.core.config;

public record CoreSwerveConfig(
    double wheelBaseMetres,
    double trackWidthMetres,
    double wheelRadiusMetres,
    double robotPeriodSeconds,
    double maxAttainableModuleSpeedMetresPerSecond,
    double maxAttainableTranslationSpeedMetresPerSecond,
    double maxAchievableRotationalVelocityRadiansPerSecond,
    ModuleConfig frontLeftModuleConfig,
    ModuleConfig frontRightModuleConfig,
    ModuleConfig backLeftModuleConfig,
    ModuleConfig backRightModuleConfig) {
}
