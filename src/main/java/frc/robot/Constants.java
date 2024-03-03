// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double ROBOT_MASS = Units.lbsToKilograms(56); // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(4)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final class Auton {

        public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.75, 0, 0);

        public static final double MAX_ACCELERATION = 2;
    }

    public static final class Drivebase {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static final class OperatorConstants {

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.01;
        public static final double LEFT_Y_DEADBAND = 0.01;
        public static final double RIGHT_X_DEADBAND = 0.01;
        public static final double TURN_CONSTANT = 6;
    }

    public static final class ArmConstants {
        public static final class RotationConstants {
            public static final int kRotateMotorPort = 14;

            public static final double BOX_TO_COG_ANGLE = 90 - 13;

            public static final double FLOOR_RESTING_ANGLE_DEGREES = -61 + BOX_TO_COG_ANGLE;
            public static final double SPEAKER_SHOOTING_ANGLE_DEGREES = 57 + BOX_TO_COG_ANGLE;
            public static final double ROTATIONS_FROM_FLOOR_REST_TO_SPEAKER = 0.565673;

            public static final double ENCODER_READING_TO_ANGLE_CONVERSION_FACTOR = (SPEAKER_SHOOTING_ANGLE_DEGREES - FLOOR_RESTING_ANGLE_DEGREES) / ROTATIONS_FROM_FLOOR_REST_TO_SPEAKER;

            public static final int HOLDING_ANGLE_CURRENT_LIMIT = 7;
            public static final int REGULAR_CURRENT_LIMIT = 80;
            public static final int HOLDING_ANGLE_VOLTAGE = 12;

            // Feedforward
            public static final double kGRotation = 0.75;
            public static final double kVRotation = 0.92;
            public static final double kSRotation = 0.1;
            public static final double kARotation = 0.0;

            // Feedback
            public static final double kPRotation = 0.3;
            public static final double kIRotation = 0.0;
            public static final double kDRotation = 0.0;

            public static final double kMaxRotationVelocityRadiansPerSecond = 2.09;
            public static final double kMaxRotationAccelerationRadiansPerSecondSquared = 5.02;

            public static final double ROTATION_FINISHED_THRESHOLD_RADIANS = Units.degreesToRadians(5);

            public static final double SLOW_DRIVING_ANGLE_THRESHOLD_DEGREES = 15 + BOX_TO_COG_ANGLE;
        }

        public static final class ShooterConstants {
            public static final int kStorageMotorPort = 45;
            public static final int kShootingMotorPort = 15;

            public static final double TIME_UNTIL_FEED = 0.7;
            public static final double LAUNCH_RUN_TIME = TIME_UNTIL_FEED + .75; //change once tested

            public static final double STORAGE_INTAKE_VOLTAGE = -3.0;
            public static final double SHOOTING_INTAKE_VOLTAGE = 6.0;

            public static final double STORAGE_SPEAKER_SHOOTER_VOLTAGE = 10;
            public static final double SHOOTING_SPEAKER_SHOOTER_VOLTAGE = -10;

            public static final double STORAGE_TRAP_SHOOTER_VOLTAGE = 6;
            public static final double SHOOTING_TRAP_SHOOTER_VOLTAGE = -7;

        }
    }

    public static class ClimbConstants {
        public static final int kClimbMotorRightPort = 9;
        public static final int kClimbMotorLeftPort = 10;

        public static final int kClimbSensorLeftUpPort = 4;
        public static final int kClimbSensorRightUpPort = 2;
        public static final int kClimbSensorLeftDownPort = 5;
        public static final int kClimbSensorRightDownPort = 3;
    }
}
