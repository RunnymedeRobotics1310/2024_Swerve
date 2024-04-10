package frc.robot.subsystems.swerve;

import ca.team1310.swervedrive.core.config.CoreSwerveConfig;
import ca.team1310.swervedrive.vision.VisionConfig;

public record SwerveDriveSubsystemConfig(
    boolean enabled,
    double robotPeriod,
    CoreSwerveConfig coreConfig,
    VisionConfig visionConfig,
    SwerveTranslationConfig translationConfig,
    SwerveRotationConfig rotationConfig) {
}
