// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final double ROBOT_MASS = (146.9) * 0.453592; // weight measured on 3/13 * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
	public static final int NUM_LEDS = 90;
	public static final double DOUBLE_NOTE_LENGTH = 2;
	public static boolean isBlueAlliance = true;

	public static final class Arm {
		public static final double SHOOTER_ELEVATOR_TOLERANCE = 5;
		public static final double SHOOTER_FLYWHEEL_TOLERANCE_RPM = 10;
		public static final double SHOOTER_FLYWHEEL_TOLERANCE_METERS = 0.2;
		public static final double SHOOTER_PIVOT_TOLERANCE = 0.009;
	}

	public static final class Climb {
		public static final double CLIMB_TOLERANCE = 0.1;
		public static final double MANUAL_DEADBAND = 0.06;
	}

	public static final class Auton {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(3, 0.0, 0.0);
		public static final PIDConstants ANGLE_AUTO_PID = new PIDConstants(3, 0, 0);

		public static final double MAX_ACCELERATION = 2;
		public static final double MAX_MODULE_SPEED = 10;
		public static final double FLYWHEEL_TOLERANCE = 0.08;
		public static final double PIVOT_TOLERANCE = 0.005;
		public static final double ROTATION_TOLERANCE = 0.05;
	}

	public static final class Drivebase {
		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
		public static final PIDController TAREGET_ANGLE_CONTROLLER = new PIDController(10, 0, 0.4);
		public static final double TURN_TO_ANGLE_TOLERANCE = 0.2;
		public static final double TURN_TO_ANGLE_TIME_TOLERANCE = 0.2;
		public static final PIDController DRIVER_TAREGET_ANGLE_CONTROLLER = new PIDController(6, 0, 1);
	}

	public static class OperatorConstants {
		// Joystick Deadband
		public static final double LEFT_X_DEADBAND = 0.07;
		public static final double LEFT_Y_DEADBAND = 0.07;
		public static final double RIGHT_X_DEADBAND = 0.07;
		public static final double TURN_CONSTANT = 6;
		public static final int DRIVER_LEFT = 0;
		public static final int DRIVER_RIGHT = 1;
		public static final int DRIVER_BUTTONS = 2;
		public static final int CO_DRIVER_LEFT = 3;
		public static final int CO_DRIVER_RIGHT = 4;
		public static final int CO_DRIVER_BUTTONS = 5;
	}

	public static class FieldConstants {
		public static final double LENGTH = Units.feetToMeters(54);
		public static final double WIDTH = Units.feetToMeters(27);
	}

	public static class PoseEstimator {
		public static final double BUFFER_DURATION_SECS = 1.5;
	}

	public static final class VisionConstants {
		// All these robot to camera are converted to meters
		private final static double ROBOT_TO_CAM_X = 0.33;
		private final static double ROBOT_TO_CAM_Y = 0.273;
		private final static double ROBOT_TO_CAM_Z = 0.3937;

		public final static boolean USE_VISION = true;

		final static double back_cam_x = 13.97;

		/*
		 * X positive is forward from the front of the robot
		 * Y positive is left from the front of the robot
		 * Z positive is upwards
		 * Positive roll is unknown because we didn't use it
		 * Positive pitch is towards the front of the robot (CCW positive around the Y
		 * axis)
		 * Positive yaw is unknown because we didn't use it
		 * Order of operations for rotations, first it rotates around the yaw, then it
		 * does pitch
		 */

		public static final Transform3d ROBOT_TO_CLYDE = new Transform3d(
				new Translation3d(0.15, 0.29, 0.37),
				new Rotation3d(Math.toRadians(0), Math.toRadians(-14.5), Math.toRadians(15)));

		public static final Transform3d ROBOT_TO_BLINKY = new Transform3d(
				new Translation3d(0.15, -0.29, 0.37),
				new Rotation3d(Math.toRadians(0), Math.toRadians(-13.5), Math.toRadians(345)));

		public static final Transform3d ROBOT_TO_PINKY = new Transform3d(
				new Translation3d(-.32, 0.28, 0.35),
				new Rotation3d(0, Math.toRadians(-13), Math.toRadians(165)));

		public static final Transform3d ROBOT_TO_INKY = new Transform3d(
				new Translation3d(-.32, -0.28, 0.35),
				new Rotation3d(0, Math.toRadians(-13), Math.toRadians(195)));

		/** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
		public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
		public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
		public static final double NOISY_DISTANCE_METERS = 2.5;
		public static final double DISTANCE_WEIGHT = 30;
		public static final int TAG_PRESENCE_WEIGHT = 10;
	}
}
