// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import ca.team1310.swervedrive.SwerveTelemetry;
import frc.robot.subsystems.swerve.SwerveDriveSubsystemConfig;
import ca.team1310.swervedrive.core.config.CoreSwerveConfig;
import ca.team1310.swervedrive.core.config.EncoderConfig;
import ca.team1310.swervedrive.core.config.ModuleConfig;
import ca.team1310.swervedrive.core.config.MotorConfig;
import ca.team1310.swervedrive.vision.VisionConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.swerve.SwerveRotationConfig;
import frc.robot.subsystems.swerve.SwerveTranslationConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OiConstants {

        public static final int    DRIVER_CONTROLLER_PORT   = 0;
        public static final int    OPERATOR_CONTROLLER_PORT = 1;

        /**
         * Standard drive speed factor. Regular teleop drive will use this factor of the max
         * translational speed.
         */
        public static final double GENERAL_SPEED_FACTOR     = .6;

        /**
         * Maximum drive speed factor. When boosting, this factor will be multiplied against the
         * max translational speed.
         */
        public static final double MAX_SPEED_FACTOR         = 1;

        /**
         * Slow mode drive speed factor. When running in slow mode, this factor will be
         * multiplied against the max translational speed.
         */
        public static final double SLOW_SPEED_FACTOR        = .1;

    }

    public static final class FieldConstants {
        public static final double FIELD_EXTENT_METRES_Y = 8.211;
        public static final double FIELD_EXTENT_METRES_X = 16.541;
    }

    public static final class Swerve {

        /**
         * Front to back from the middle of the wheels
         */
        public static final double                     WHEEL_BASE_METRES         = inchesToMeters(21);
        /**
         * Side to side from the middle of the wheels
         */
        public static final double                     TRACK_WIDTH_METRES        = inchesToMeters(21.75);

        public static final double                     SDS_MK4I_WHEEL_RADIUS_M   = 0.0051;

        private static final int                       ANGLE_ENCODER_UPDATE_FREQ = 10;

        public static final SwerveTranslationConfig    TRANSLATION_CONFIG        = new SwerveTranslationConfig(
            0.02,
            1.0,
            4.79,
            4.79,
            12.0,
            1.2, 0, 0);

        public static final SwerveRotationConfig       ROTATION_CONFIG           = new SwerveRotationConfig(
            /* min rot vel radPS */ Rotation2d.fromDegrees(10).getRadians(),
            /* max rot vel radPS */ Rotation2d.fromRotations(1).getRadians(),
            /* max rotation jump speed */ Rotation2d.fromDegrees(205).getRadians(),
            /* slow zone */ Rotation2d.fromDegrees(35).getRadians(),
            /* max rotation accel */ Rotation2d.fromRotations(1310).getRadians(),
            /* rotation tolerance */ Rotation2d.fromDegrees(2).getRadians(),
            0.8, 0, 0);


        private static final MotorConfig               ANGLE_MOTOR_CONFIG        = new MotorConfig(
            true,
            20, 12,
            0.25,
            150.0 / 7 /* SDS MK4i 150/7:1 */,
            0.0125, 0, 0, 0, 0);

        private static final MotorConfig               DRIVE_MOTOR_CONFIG        = new MotorConfig(
            true,
            40, 12,
            0.25, 6.75 /* SDS MK4i L2 --> 6.75:1 */,
            0.11, 0, 0, 0, 0);

        private static final EncoderConfig             ANGLE_ENCODER_CONFIG      = new EncoderConfig(false, 0.005, 5);

        public static final ModuleConfig               FRONT_LEFT                = new ModuleConfig(
            "frontleft",
            TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2,
            SDS_MK4I_WHEEL_RADIUS_M,
            10, DRIVE_MOTOR_CONFIG,
            11, ANGLE_MOTOR_CONFIG,
            12, Rotation2d.fromRotations(0.281494).getDegrees(),
            ANGLE_ENCODER_CONFIG, ANGLE_ENCODER_UPDATE_FREQ);


        public static final ModuleConfig               FRONT_RIGHT               = new ModuleConfig(
            "frontright",
            TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2,
            SDS_MK4I_WHEEL_RADIUS_M,
            20, DRIVE_MOTOR_CONFIG,
            21, ANGLE_MOTOR_CONFIG,
            22, Rotation2d.fromRotations(0.407959).getDegrees(),
            ANGLE_ENCODER_CONFIG, ANGLE_ENCODER_UPDATE_FREQ);

        public static final ModuleConfig               BACK_LEFT                 = new ModuleConfig(
            "backleft",
            -TRACK_WIDTH_METRES / 2, WHEEL_BASE_METRES / 2,
            SDS_MK4I_WHEEL_RADIUS_M,
            35, DRIVE_MOTOR_CONFIG,
            36, ANGLE_MOTOR_CONFIG,
            37, Rotation2d.fromRotations(0.503418).getDegrees(),
            ANGLE_ENCODER_CONFIG, ANGLE_ENCODER_UPDATE_FREQ);

        public static final ModuleConfig               BACK_RIGHT                = new ModuleConfig(
            "backright",
            -TRACK_WIDTH_METRES / 2, -WHEEL_BASE_METRES / 2,
            SDS_MK4I_WHEEL_RADIUS_M,
            30, DRIVE_MOTOR_CONFIG,
            31, ANGLE_MOTOR_CONFIG,
            32, Rotation2d.fromRotations(0.359131).getDegrees(),
            ANGLE_ENCODER_CONFIG, ANGLE_ENCODER_UPDATE_FREQ);

        public static final SwerveTelemetry            TELEMETRY                 = new SwerveTelemetry(4);

        public static final CoreSwerveConfig           CORE_SWERVE_CONFIG        = new CoreSwerveConfig(
            WHEEL_BASE_METRES, TRACK_WIDTH_METRES, SDS_MK4I_WHEEL_RADIUS_M,
            Robot.kDefaultPeriod,
            TRANSLATION_CONFIG.maxModuleSpeedMPS(),
            TRANSLATION_CONFIG.maxSpeedMPS(),
            ROTATION_CONFIG.maxRotVelocityRadPS(),
            FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT,
            TELEMETRY);

        public static final VisionConfig               VISION_CONFIG             = new VisionConfig(
            0, 0,
            FieldConstants.FIELD_EXTENT_METRES_X, FieldConstants.FIELD_EXTENT_METRES_Y,
            0.7, 0.1, .5);

        public static final SwerveDriveSubsystemConfig SUBSYSTEM_CONFIG          = new SwerveDriveSubsystemConfig(
            true, CORE_SWERVE_CONFIG, VISION_CONFIG, TRANSLATION_CONFIG, ROTATION_CONFIG);
    }

    public enum BotTarget {

        // Blue Field Targets
        BLUE_AMP(new Translation3d(1.8415, 8.2042, 0.873252)),
        BLUE_SOURCE(new Translation3d(15.632176, 0.564896, 0)),
        BLUE_SPEAKER(new Translation3d(0.0381, 5.547868, 2.124202)),
        BLUE_STAGE(new Translation3d(4.86791, 4.105656, 1.6764)),

        // Red Field Targets
        RED_AMP(new Translation3d(14.700758, 8.2042, 0.873252)),
        RED_SOURCE(new Translation3d(0.908812, 0.564769, 0)),
        RED_SPEAKER(new Translation3d(16.579342, 5.547868, 2.124202)),
        RED_STAGE(new Translation3d(11.676634, 4.105656, 1.6764)),

        // Blue Side Notes
        BLUE_NOTE_WOLVERINE(new Translation3d(2.9, 4.11, 0)),
        BLUE_NOTE_BARNUM(new Translation3d(2.9, 5.5, 0)),
        BLUE_NOTE_VALJEAN(new Translation3d(2.9, 7, 0)),

        // Red Side Notes
        RED_NOTE_WOLVERINE(new Translation3d(13.53, 4.11, 0)),
        RED_NOTE_BARNUM(new Translation3d(13.53, 5.5, 0)),
        RED_NOTE_VALJEAN(new Translation3d(13.53, 7, 0)),

        // Centre Field Notes
        CENTRE_NOTE_1(new Translation3d(8.16, 0.75, 0)),
        CENTRE_NOTE_2(new Translation3d(8.16, 2.43, 0)),
        CENTRE_NOTE_3(new Translation3d(8.16, 4.11, 0)),
        CENTRE_NOTE_4(new Translation3d(8.16, 5.79, 0)),
        CENTRE_NOTE_5(new Translation3d(8.16, 7.47, 0)),

        // When No Target is Set
        NONE(new Translation3d(0, 0, 0)),

        // No focus, but go to any tag visible
        ALL(new Translation3d(0, 0, 0));


        private final Translation3d location;

        BotTarget(Translation3d location) {
            this.location = location;
        }

        public Translation3d getLocation() {
            return location;
        }

        @Override
        public String toString() {
            return "BotTarget: " + name() + " at " + location;
        }
    }

    public static final class UsefulPoses {

        public static final Pose2d SCORE_BLUE_AMP = (new Pose2d(BotTarget.BLUE_AMP.getLocation().getX(), 7.6,
            Rotation2d.fromDegrees(90)));
        public static final Pose2d SCORE_RED_AMP  = (new Pose2d(BotTarget.RED_AMP.getLocation().getX(), 7.6,
            Rotation2d.fromDegrees(90)));

        public static final Pose2d BLUE_2_2_20    = new Pose2d(2, 2, Rotation2d.fromDegrees(20));
        public static final Pose2d RED_2_2_20     = new Pose2d(14.54, 2, Rotation2d.fromDegrees(-20));

    }
}
