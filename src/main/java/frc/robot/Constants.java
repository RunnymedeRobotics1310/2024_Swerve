// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.subsystems.vision.PoseConfidence.HIGH;
import static frc.robot.subsystems.vision.PoseConfidence.LOW;
import static frc.robot.subsystems.vision.PoseConfidence.MEDIUM;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.PoseConfidence;

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

        public static final int DRIVER_CONTROLLER_PORT   = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class Swerve {

        public static final class Chassis {

            public static final double     SDS_MK4I_WHEEL_RADIUS_METRES           = 0.0051;
            /**
             * Specify the maximum speed a module can physically reach in m/s.
             * The SDS
             * <a href="https://www.swervedrivespecialties.com/products/mk4i-swerve-module">MK4i</a>
             * module with L2 gear ratio supports a maximum drive motor speed of 15.7ft/s (4.79m/s).
             * 
             * Do not use this value in software to cap how fast the robot drives on the field.
             * For that, use {@link #MAX_TRANSLATION_SPEED_MPS}.
             */
            public static final double     MAX_MODULE_SPEED_MPS                   = 4.79;
            /**
             * Set how fast you want the robot to actually translate across the field.
             * This is the "speed limit" of the robot.
             *
             * Practically speaking 4.42 m/s is a good max, but
             * consider 1-2 for development and 2-3 for competitions.
             */
            public static final double     MAX_TRANSLATION_SPEED_MPS              = 4.42;
            public static final Rotation2d MAX_ROTATIONAL_VELOCITY_PER_SEC        = Rotation2d.fromRotations(1);
            public static final Rotation2d MIN_ROTATIONAL_VELOCITY_PER_SEC        = Rotation2d.fromDegrees(10);
            public static final Rotation2d ROTATION_TOLERANCE                     = Rotation2d.fromDegrees(1);
            public static final double     TRANSLATION_TOLERANCE_METRES           = 0.02;
            public static final double     DECEL_FROM_MAX_TO_STOP_DIST_METRES     = 1.9;
            public static final double     MAX_ROTATION_ACCELERATION_RAD_PER_SEC2 = Rotation2d.fromRotations(800).getRadians();
            public static final double     MAX_TRANSLATION_ACCELERATION_MPS2      = 8;

            /**
             * Standard drive speed factor. Regular teleop drive will use this factor of the max
             * translational speed.
             */
            public static final double     GENERAL_SPEED_FACTOR                   = .5;

            /**
             * Maximum drive speed factor. When boosting, this factor will be multiplied against the
             * max translational speed.
             * todo: tune
             */
            public static final double     MAX_SPEED_FACTOR                       = 1;

            /**
             * Slow mode drive speed factor. When running in slow mode, this factor will be
             * multiplied against the max translational speed.
             * todo: tune
             */
            public static final double     SLOW_SPEED_FACTOR                      = .1;

            public static final class HeadingPIDConfig {
                public static final double P = 0.8;
                // .002 is too low but stable
                public static final double I = 0;
                public static final double D = 0;
            }

            public static final class VelocityPIDConfig {
                // public static final double P = 15;
                // public static final double P = 1.5;
                public static final double P = 1.2;
                // .002 is too low but stable
                public static final double I = 0;
                public static final double D = 0;

            }
        }

        public static final class Motor {
            public boolean            inverted;
            public int                currentLimitAmps;
            public double             nominalVoltage;
            public double             rampRate;
            public double             gearRatio;
            public double             p;
            public double             i;
            public double             d;
            public double             ff;
            public double             iz;
            public static final Motor DRIVE = new Motor();

            static {
                DRIVE.inverted         = true;
                DRIVE.currentLimitAmps = 40;
                DRIVE.nominalVoltage   = 12;
                DRIVE.rampRate         = 0.25;
                DRIVE.gearRatio        = 6.75;     // SDS MK4i L2 --> 6.75:1
                DRIVE.p                = 0.0020645;
                DRIVE.i                = 0;
                DRIVE.d                = 0;
                DRIVE.ff               = 0;
                DRIVE.iz               = 0;
            }

            public static final Motor ANGLE = new Motor();

            static {
                ANGLE.inverted         = true;
                ANGLE.currentLimitAmps = 20;        // must not exceed 30 (fuse)
                ANGLE.nominalVoltage   = 12;
                ANGLE.rampRate         = 0.25;
                ANGLE.gearRatio        = 150.0 / 7; // SDS MK4i 150/7:1
                ANGLE.p                = 0.01;
                ANGLE.i                = 0;
                ANGLE.d                = 0;
                ANGLE.ff               = 0;
                ANGLE.iz               = 0;
            }
        }

        public static final class Module {
            public double              wheelRadiusMetres;
            public Translation2d       locationMetres;
            public int                 driveCANID;
            public int                 angleCANID;
            public int                 encoderCANID;
            public double              encoderAbsoluteOffsetDegrees;
            public static final Module BACK_LEFT = new Module();

            static {
                BACK_LEFT.wheelRadiusMetres            = Chassis.SDS_MK4I_WHEEL_RADIUS_METRES;
                BACK_LEFT.locationMetres               = new Translation2d(inchesToMeters(-11.375), inchesToMeters(10.875));
                BACK_LEFT.driveCANID                   = 35;
                BACK_LEFT.angleCANID                   = 36;
                BACK_LEFT.encoderCANID                 = 37;
                BACK_LEFT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.503418).getDegrees();
            }

            public static final Module BACK_RIGHT = new Module();

            static {
                BACK_RIGHT.wheelRadiusMetres            = Chassis.SDS_MK4I_WHEEL_RADIUS_METRES;
                BACK_RIGHT.locationMetres               = new Translation2d(inchesToMeters(-11.375), inchesToMeters(-10.875));
                BACK_RIGHT.driveCANID                   = 30;
                BACK_RIGHT.angleCANID                   = 31;
                BACK_RIGHT.encoderCANID                 = 32;
                BACK_RIGHT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.359131).getDegrees();
            }

            public static final Module FRONT_LEFT = new Module();

            static {
                FRONT_LEFT.wheelRadiusMetres            = Chassis.SDS_MK4I_WHEEL_RADIUS_METRES;
                FRONT_LEFT.locationMetres               = new Translation2d(inchesToMeters(11.375), inchesToMeters(10.875));
                FRONT_LEFT.driveCANID                   = 10;
                FRONT_LEFT.angleCANID                   = 11;
                FRONT_LEFT.encoderCANID                 = 12;
                FRONT_LEFT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.281494).getDegrees();
            }

            public static final Module FRONT_RIGHT = new Module();

            static {
                FRONT_RIGHT.wheelRadiusMetres            = Chassis.SDS_MK4I_WHEEL_RADIUS_METRES;
                FRONT_RIGHT.locationMetres               = new Translation2d(inchesToMeters(11.375), inchesToMeters(-10.875));
                FRONT_RIGHT.driveCANID                   = 20;
                FRONT_RIGHT.angleCANID                   = 21;
                FRONT_RIGHT.encoderCANID                 = 22;
                FRONT_RIGHT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.407959).getDegrees();
            }
        }
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

        public static final Pose2d SCORE_BLUE_AMP = (new Pose2d(BotTarget.BLUE_AMP.getLocation().getX(), 7.8,
            Rotation2d.fromDegrees(90)));
        public static final Pose2d SCORE_RED_AMP  = (new Pose2d(BotTarget.RED_AMP.getLocation().getX(), 7.8,
            Rotation2d.fromDegrees(90)));

        public static final Pose2d BLUE_2_2_20    = new Pose2d(2, 2, Rotation2d.fromDegrees(20));
        public static final Pose2d RED_2_2_20     = new Pose2d(14.54, 2, Rotation2d.fromDegrees(-20));

    }


    public static final class VisionConstants {
        /** Time to switch pipelines and acquire a new vision target */
        public static final double  VISION_SWITCH_TIME_SEC         = .25;

        // todo: correct this
        public static Translation2d CAMERA_LOC_REL_TO_ROBOT_CENTER = new Translation2d(0, 30);

        /**
         * Utility method (STATIC) to map confidence and pose difference to a matrix of estimated
         * standard
         * deviations. The returned matrix values have been tuned based on the input and are not
         * dynamically calculated.
         *
         * @param confidence rating from the vision subsystem
         * @param poseDifferenceMetres difference between estimated pose and pose from vision
         * @return matrix of standard deviations, or null if values are too far out of bounds
         */
        public static Matrix<N3, N1> getVisionStandardDeviation(PoseConfidence confidence, double poseDifferenceMetres) {
            double xyMetresStds;
            double degreesStds;

            // todo: measure / tune these values
            if (confidence == HIGH) {
                xyMetresStds = 0.05;
                degreesStds  = 2;
            }
            else if (confidence == MEDIUM || poseDifferenceMetres < 0.5) {
                xyMetresStds = 0.15;
                degreesStds  = 6;
            }
            else if (confidence == LOW || poseDifferenceMetres < 0.8) {
                xyMetresStds = 0.30;
                degreesStds  = 12;
            }
            else { // Covers the Confidence.NONE case
                return null;
            }

            return VecBuilder.fill(xyMetresStds, xyMetresStds, Units.degreesToRadians(degreesStds));
        }
    }

    public static final class AutoConstants {

        public static enum AutoPattern {
            SCORE_1_AMP, SCORE_2_AMP, SCORE_1_SPEAKER, SCORE_3_SPEAKER, SCORE_4_SPEAKER
        }
    }
}
