// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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

            public static final double SDS_MK4I_WHEEL_RADIUS_METRES           = 0.0051;
            /**
             * Specify the maximum speed a module can reach in m/s. This should be measured
             * so that we don't ask the module to do something it's not capable of doing. This
             * is NOT a value we should use to cap how fast the robot drives on the field. For
             * that, use {@link #MAX_TRANSLATION_SPEED_MPS}.
             */
            public static final double MAX_MODULE_SPEED_MPS                   = 5;
            /**
             * Set how fast you want the robot to actually translate across the field.
             * This is the "speed limit" of the robot.
             *
             * Practically speaking 4.42 m/s is a good max, but
             * consider 1-2 for development and 2-3 for competitions.
             */
            public static final double MAX_TRANSLATION_SPEED_MPS              = 4.42;
            public static final double MAX_ROTATIONAL_VELOCITY_RAD_PER_SEC    = Rotation2d.fromRotations(2).getRadians();
            public static final double ROTATION_TOLERANCE_RADIANS             = Rotation2d.fromDegrees(0.5).getRadians();
            public static final double MAX_ROTATION_ACCELERATION_RAD_PER_SEC2 = Rotation2d.fromRotations(4).getRadians();
            public static final double MAX_TRANSLATION_ACCELERATION_MPS2      = 15;

            /**
             * Standard drive speed factor. Regular teleop drive will use this factor of the max
             * translational speed.
             */
            public static final double GENERAL_SPEED_FACTOR                   = .5;

            /**
             * Maximum drive speed factor. When boosting, this factor will be multiplied against the
             * max translational speed.
             * todo: tune
             */
            public static final double MAX_SPEED_FACTOR                       = 1;

            /**
             * Slow mode drive speed factor. When running in slow mode, this factor will be
             * multiplied against the max translational speed.
             * todo: tune
             */
            public static final double SLOW_SPEED_FACTOR                      = .1;

            public static final class HeadingPIDConfig {
                public static final double P = 0.3;
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
                BACK_LEFT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.504883).getDegrees();
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
                FRONT_LEFT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.154297).getDegrees();
            }

            public static final Module FRONT_RIGHT = new Module();

            static {
                FRONT_RIGHT.wheelRadiusMetres            = Chassis.SDS_MK4I_WHEEL_RADIUS_METRES;
                FRONT_RIGHT.locationMetres               = new Translation2d(inchesToMeters(11.375), inchesToMeters(-10.875));
                FRONT_RIGHT.driveCANID                   = 20;
                FRONT_RIGHT.angleCANID                   = 21;
                FRONT_RIGHT.encoderCANID                 = 22;
                FRONT_RIGHT.encoderAbsoluteOffsetDegrees = Rotation2d.fromRotations(0.414795).getDegrees();
            }
        }
    }

    public final class VisionConstants {

        /** Time to switch pipelines and acquire a new vision target */
        public static final double VISION_SWITCH_TIME_SEC = .25;

        public enum VisionTarget {
            SPEAKER,
            AMP,
            SOURCE,
            STAGE,
            ROBOT,
            APRILTAGS,
            NOTE,
            NONE;

            @Override
            public String toString() {
                return "VisionTarget: " + name();
            }

        }

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
        public static Matrix<N3, N1> getVisionStandardDeviation(double confidence, double poseDifferenceMetres) {
            double xyMetresStds;
            double degreesStds;

            // todo: measure / tune these values
            if (confidence > 0.8) {
                xyMetresStds = 0.5;
                degreesStds  = 6;
            }
            else if (poseDifferenceMetres < 0.5) {
                xyMetresStds = 1.0;
                degreesStds  = 12;
            }
            else if (poseDifferenceMetres < 0.8) {
                xyMetresStds = 2.0;
                degreesStds  = 24;
            }
            else {
                return null;
            }

            return VecBuilder.fill(xyMetresStds, xyMetresStds, Units.degreesToRadians(degreesStds));
        }
    }
}
