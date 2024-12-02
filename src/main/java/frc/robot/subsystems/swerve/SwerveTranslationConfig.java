package frc.robot.subsystems.swerve;

/**
 * Configure the translation configuration properties of this swerve drive.
 * 
 * @param toleranceMetres The tolerance in meters for the translation. If the robot is within this
 * distance of the target, it is considered on target.
 * @param minSpeedMPS The minimum speed the robot will move at. This is to prevent the robot from
 * moving too slowly.
 * @param maxSpeedMPS Set how fast you want the robot to actually translate across the field.
 * This is the "speed limit" of the robot.
 * <p>
 * Practically speaking 4.42 m/s is a good max, but
 * consider 1-2 for development and 2-3 for competitions.
 * @param maxModuleSpeedMPS Specify the maximum speed a module can physically reach in m/s.
 * The SDS
 * <a href="https://www.swervedrivespecialties.com/products/mk4i-swerve-module">MK4i</a>
 * module with L2 gear ratio supports a maximum drive motor speed of 15.7ft/s (4.79m/s).
 * <p>
 * Do not use this value in software to cap how fast the robot drives on the field.
 * For that, use {@link #maxAccelMPS2}
 * @param maxAccelMPS2 The maximum acceleration the robot will move at. This is to prevent the robot
 * from accelerating dangerously quickly
 * @param velocityP The proportional gain for the velocity PID controller
 * This is the "strength" of the controller. Higher values will make the robot more aggressive.
 * @param velocityI The integral gain for the velocity PID controller
 * This is the "memory" of the controller. It will remember past errors and use them to correct the
 * robot's path.
 * @param velocityD The derivative gain for the velocity PID controller
 * This is the "prediction" of the controller. It will predict future errors and use them to correct
 * the robot's path.
 */
public record SwerveTranslationConfig(
    double toleranceMetres,
    double minSpeedMPS,
    double maxSpeedMPS,
    double maxModuleSpeedMPS,
    double maxAccelMPS2,
    double velocityP,
    double velocityI,
    double velocityD) {
}
