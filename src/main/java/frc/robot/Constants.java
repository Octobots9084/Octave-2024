package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

        // public static final class DrivetrainConstants {
        //         /**
        //         * The left-to-right distance between the drivetrain wheels
        //         * <p>
        //         * Should be measured from center to center.
        //         */
        //         public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.3556;
        //         /**
        //          * The front-to-back distance between the drivetrain wheels.
        //          * <p>
        //          * Should be measured from center to center.
        //          */
        //         public static final double DRIVETRAIN_WHEELBASE_METERS = 0.3556;

        //         public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        //                         // Front left
        //                         new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        //                                         DRIVETRAIN_WHEELBASE_METERS / 2.0),
        //                         // Front right
        //                         new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        //                                         -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        //                         // Back left
        //                         new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        //                                         DRIVETRAIN_WHEELBASE_METERS / 2.0),
        //                         // Back right
        //                         new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
        //                                         -DRIVETRAIN_WHEELBASE_METERS / 2.0));
        // }

        public static final class VisionConstants {
                public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
                public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);

                public static boolean USE_VISION = true;

                /**
                 * Physical location of the front right camera on the robot, relative to the center of
                 * the
                 * robot.
                 */
                public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
                                new Translation3d(Units.inchesToMeters(-11.88), Units.inchesToMeters(-6.88),
                                                Units.inchesToMeters(31.09)),
                                new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(-45)));

                /**
                 * Physical location of the front left camera on the robot, relative to the center of
                 * the
                 * robot.
                 */
                public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
                                new Translation3d(Units.inchesToMeters(-11.88), Units.inchesToMeters(6.88),
                                                Units.inchesToMeters(31.09)),
                                new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(46)));

                /**
                 * Physical location of the back right camera on the robot, relative to the center of
                 * the
                 * robot.
                 */
                public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA = new Transform3d(
                                new Translation3d(Units.inchesToMeters(-13.76), Units.inchesToMeters(6.1),
                                                Units.inchesToMeters(33.65)),
                                new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(180)));

                /**
                 * Physical location of the back left camera on the robot, relative to the center of
                 * the
                 * robot.
                 */
                public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(
                                new Translation3d(Units.inchesToMeters(-13.76), Units.inchesToMeters(6.1),
                                                Units.inchesToMeters(33.65)),
                                new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(180)));

                /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
                public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
                public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
                public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
                public static final double NOISY_DISTANCE_METERS = 2.5;
                public static final double DISTANCE_WEIGHT = 7;
                public static final int TAG_PRESENCE_WEIGHT = 10;

                /**
                 * Standard deviations of model states. Increase these numbers to trust your
                 * model's state estimates less. This
                 * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
                 * meters.
                 * If these numbers are less than one, multiplying will do bad things
                 */

                public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(),
                                Nat.N1(), 1, // x
                                1, // y
                                1 * Math.PI); // theta

                /**
                 * Standard deviations of the vision measurements. Increase these numbers to
                 * trust global measurements from vision
                 * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
                 * radians.
                 * If these numbers are less than one, multiplying will do bad things
                 */
                public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(),
                                .1, // x
                                .1, // y 
                                .1);
        }
}
