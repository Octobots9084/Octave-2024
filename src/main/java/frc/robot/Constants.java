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

	public static final double ROBOT_MASS = (65) * 0.453592; // 32lbs * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

	public static final class Auton {
		public static final PIDConstants TranslationPID = new PIDConstants(3, 0.0, 0.0);
		public static final PIDConstants angleAutoPID = new PIDConstants(2, 0, 0);

		public static final double MAX_ACCELERATION = 2;
	}

	public static final class Drivebase {
		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
		public static PIDController targetAngleController = new PIDController(10, 0, 0);
	}

	public static class OperatorConstants {
		// Joystick Deadband
		public static final double LEFT_X_DEADBAND = 0.07;
		public static final double LEFT_Y_DEADBAND = 0.07;
		public static final double RIGHT_X_DEADBAND = 0.07;
		public static final double TURN_CONSTANT = 6;
		public static final int DRIVER_LEFT = 0;
		public static final int DRIVER_RIGHT = 1;
	}

	public static class FieldConstants {
		public static final double LENGTH = Units.feetToMeters(54);
		public static final double WIDTH = Units.feetToMeters(27);
	}

	public static final class VisionConstants {
		// All these robot to camera are converted to meters and divided in half
		private static double ROBOT_TO_CAM_X = (53.0 / 100) / 2;
		private static double ROBOT_TO_CAM_Y = (53.0 / 100) / 2;
		private static double ROBOT_TO_CAM_Z = (36.0 / 100);

		public static boolean USE_VISION = false;

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
				new Translation3d(-ROBOT_TO_CAM_X, ROBOT_TO_CAM_Y, ROBOT_TO_CAM_Z),
				new Rotation3d(Math.toRadians(0), Math.toRadians(-11), Math.toRadians(180)));

		public static final Transform3d ROBOT_TO_PINKY = new Transform3d(
				new Translation3d(-ROBOT_TO_CAM_X, -ROBOT_TO_CAM_Y, ROBOT_TO_CAM_Z),
				new Rotation3d(Math.toRadians(0), Math.toRadians(-13), Math.toRadians(180)));

		// public static final Transform3d ROBOT_TO_BLINKY = new Transform3d(
		// 		new Translation3d(ROBOT_TO_CAM_X, ROBOT_TO_CAM_Y, ROBOT_TO_CAM_Z),
		// 		new Rotation3d(0, Math.toRadians(-9.7), Math.toRadians(0)));

		// public static final Transform3d ROBOT_TO_INKY = new Transform3d(
		// 		new Translation3d(ROBOT_TO_CAM_X, -ROBOT_TO_CAM_Y, ROBOT_TO_CAM_Z),
		// 		new Rotation3d(0, Math.toRadians(-10.1), Math.toRadians(0)));

		/** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
		public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
		public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
		public static final double NOISY_DISTANCE_METERS = 2.5;
		public static final double DISTANCE_WEIGHT = 7;
		public static final int TAG_PRESENCE_WEIGHT = 10;
	}
}
