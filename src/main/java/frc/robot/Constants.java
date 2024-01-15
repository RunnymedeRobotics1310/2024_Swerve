// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OiConstants {

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class SwerveDriveConstants {
        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
        public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
        /**
         * Maximum speed of the robot in meters per second, used for both angular acceleration used in
         * swervelib.SwerveController and drive feedforward in SwerveMath#createDriveFeedforward(double, double, double).
         */
        public static final double MAX_SPEED_MPS = 4.42;
        public static final double MAX_ROTATION_RADIANS_PER_SEC = 6;

        public static final double ROTATION_TOLERANCE_DEGREES = 2.0;
        public static final double ROTATION_ANGULAR_VELOCITY_TOLERANCE_PCT = 0.08;
    }
}
