// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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

    public static final double ROBOT_MASS = Units.lbsToKilograms(113.9); // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(4)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final class AutonConstants {

        public static final PIDFConfig angleAutoPID = new PIDFConfig(2.1, 0, 0);
        // speed to move forwards when going to intake during auto
        public static final double intakeForwardsSpeedMetersPerSecond = Units.inchesToMeters(40);

        public static final double MAX_ACCELERATION = 2;

        public static final double INTAKE_TIMEOUT_SECONDS = 1;
        public static final double BACKUP_WAIT_SECONDS = 6;
    }

    public static final class DriveConstants {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final Rotation2d LEFT_SHOOTER_ANGLE = Rotation2d.fromDegrees(-60);
        public static final Rotation2d CENTER_SHOOTER_ANGLE = Rotation2d.fromDegrees(180);
        public static final Rotation2d RIGHT_SHOOTER_ANGLE = Rotation2d.fromDegrees(60);

        public final static double SLOW_SPEED_SCALAR = 0.8;
    }

    public static final class OperatorConstants {
        public static final double LEFT_X_DEADBAND = 0.01;
        public static final double LEFT_Y_DEADBAND = 0.01;
        public static final double RIGHT_X_DEADBAND = 0.01;
        public static final double TURN_CONSTANT = 6;

        public static final double RUMBLE_POWER_PERCENTAGE = 0.5;
        public static final double RUMBLE_TIME_SECONDS = 2.0;

    }

    public static final class ArmConstants {
        public static final class RotationConstants {
            public static final int kRotateMotorPort = 14;

            public static final double BOX_TO_COG_ANGLE = 90 - 13;

            public static final double FLOOR_RESTING_ANGLE_DEGREES = -60 + BOX_TO_COG_ANGLE;
            public static final double SPEAKER_SHOOTING_ANGLE_DEGREES = 55.5 + BOX_TO_COG_ANGLE;
            public static final double ROTATIONS_PER_DEGREE = 0.565673/(57 - (-61));
            public static final double ROTATIONS_FROM_FLOOR_REST_TO_SPEAKER = ROTATIONS_PER_DEGREE* (57 - (-61));

            public static final double ENCODER_READING_TO_ANGLE_CONVERSION_FACTOR = (SPEAKER_SHOOTING_ANGLE_DEGREES - FLOOR_RESTING_ANGLE_DEGREES) / ROTATIONS_FROM_FLOOR_REST_TO_SPEAKER;

            public static final int HOLDING_ANGLE_CURRENT_LIMIT = 7; // TODO: Retune
            public static final int REGULAR_CURRENT_LIMIT = 80;
            public static final int HOLDING_ANGLE_VOLTAGE = 12;

            // Feedforward
            public static final double kGRotation = 0.75;
            public static final double kVRotation = 0.92;
            public static final double kSRotation = 0.3;
            public static final double kARotation = 0.0;

            // Feedback
            public static final double kPRotation = 0.2;
            public static final double kIRotation = 0.0;
            public static final double kDRotation = 0.0;

            public static final double kMaxRotationVelocityRadiansPerSecond = 2.09;
            public static final double kMaxRotationAccelerationRadiansPerSecondSquared = 5.02;

            // Distance rotation
            public static final double kPDistanceRotation = 16.0;
            public static final double kIDistanceRotation = 0.0;
            public static final double kDDistanceRotation = 1.0;

            public static final double kMaxDistanceRotationVelocityRadiansPerSecond = 2.09;
            public static final double kMaxDistanceRotationAccelerationRadiansPerSecondSquared = 5.02;
            public static final double DISTANCE_SHOOTING_ANGLE_DEGREES = 35 + BOX_TO_COG_ANGLE; // TODO: Measure
            public static final double DISTANCE_ROTATION_FINISHED_THRESHOLD_RADIANS = Units.degreesToRadians(0.1);

            public static final double ROTATION_FINISHED_THRESHOLD_RADIANS = Units.degreesToRadians(3);

            public static final double SLOW_DRIVING_ANGLE_THRESHOLD_DEGREES = 15 + BOX_TO_COG_ANGLE;
        }

        public static final class ShooterConstants {
            public static final int kStorageMotorPort = 45;
            public static final int kShootingMotorPort = 15;

            public static final double TIME_UNTIL_FEED = 0.75;
            public static final double LAUNCH_RUN_TIME = TIME_UNTIL_FEED + .5; //change once tested

            public static final double STORAGE_INTAKE_VOLTAGE = -3.0;
            public static final double SHOOTING_INTAKE_VOLTAGE = 6.0;

            public static final double STORAGE_SPEAKER_SHOOTER_VOLTAGE = 13;
            public static final double SHOOTING_SPEAKER_SHOOTER_VOLTAGE = -11;

            public static final double NOTE_CAPTURED_STALL_CURRENT_THRESHOLD = 45;
        }
    }

    public static class ClimbConstants {
        public static final int kClimbMotorRightPort = 9;
        public static final int kClimbMotorLeftPort = 10;

        public static final int kClimbSensorLeftUpPort = 4;
        public static final int kClimbSensorRightUpPort = 2;
        public static final int kClimbSensorLeftDownPort = 5;
        public static final int kClimbSensorRightDownPort = 3;

        // Separated in case one climber suffers a mechanical issue and cannot reach the same point as the other
        // TODO: Set to proper values
        public static final double CLIMB_LEFT_UP_THRESHOLD_ROTATIONS = 107.0;
        public static final double CLIMB_RIGHT_UP_THRESHOLD_ROTATIONS = 112.001221;

        public static final double CLIMB_LEFT_DOWN_THRESHOLD_ROTATIONS = 10;
        public static final double CLIMB_RIGHT_DOWN_THRESHOLD_ROTATIONS = 10;
    }
}
