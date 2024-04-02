package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

// velocities are positive going towards the target and negative when moving away
// all units should be in meters, m/s, rad, etc.
// xPos is the distance from being in line with the subwoofer
// yPos is distance from the subwoofer (including the extended base part of it)

public class ReverseKinematics {
        // distance between launcher opening and the subwoofer target
        private static double constTargetHeightDiff = 1.41;
        // gravity
        private static double g = 9.8;
        // the final y velocity for the note to be moving at when it enters the target

        // X and Y positions of the subwoofer with regards to (0,0) on the robot's
        // Pose2d
        private static double subwooferXPos = 0;
        private static double subwooferYPos = 5.5;
        private static double encoderOffset = 0.597;
        private static double movementMultiplierX = 1;
        private static double movementMultiplierY = 1;
        private static double flywheelSpeedMultiplier = 0.9;
        private static double gravityMultiplier = 0.55;
        private static double spinVComp = 0;

        // converts Pose2d coords into positions relative to the target
        public static Pose2d convert2dCoords(Pose2d pos) {
                if (Constants.isBlueAlliance) {
                        subwooferXPos = 0.22;
                        subwooferYPos = 5.554;
                } else {
                        subwooferXPos = 16.526;
                        subwooferYPos = 5.554;
                }
                // SmartDashboard.putString("poseconvert",
                // new Pose2d(pos.getX() - subwooferXPos, pos.getY() - subwooferYPos, new
                // Rotation2d())
                // .toString());
                return new Pose2d(pos.getX() - subwooferXPos, pos.getY() - subwooferYPos, new Rotation2d());
        }

        // converts ChassisSpeeds to absolute rather than relative to the robot rotation
        public static ChassisSpeeds convertSpeed(Pose2d pos, ChassisSpeeds speed) {
                return new ChassisSpeeds(
                                -(pos.getRotation().getCos() * speed.vxMetersPerSecond)
                                                - (pos.getRotation().getSin() * speed.vyMetersPerSecond),
                                (pos.getRotation().getSin() * speed.vxMetersPerSecond)
                                                + (pos.getRotation().getCos() * speed.vyMetersPerSecond)
                                                + (pos.getRotation().getCos() * spinVComp),
                                0);
        }

        // returns the vertical launch velocity of the note
        // for internal use only
        private static double calcLaunchVerticalVel(Pose2d pos, ChassisSpeeds speed, double timeInAir) {
                double verticalVel = ((constTargetHeightDiff / timeInAir) + (0.5 * g * timeInAir * gravityMultiplier)); // (pos.getY()
                                                                                                                        // /
                // timeInAir)

                // SmartDashboard.putNumber("verticalVel", verticalVel);
                return verticalVel;
        }

        // returns the horizontal (parallel to the subwoofer opening) launch velocity of
        // the note
        // for internal use only
        private static double calcLaunchXVel(Pose2d pos, ChassisSpeeds speed, double timeInAir) {
                double xVel = (pos.getX() / timeInAir) + (speed.vxMetersPerSecond * movementMultiplierX);
                // SmartDashboard.putNumber("xVel", xVel);
                return xVel;
        }

        // returns the "vertical" from above (perpendicular to the subwoofer opening)
        // launch velocity of the note
        // for internal use only
        private static double calcLaunchYVel(Pose2d pos, ChassisSpeeds speed, double timeInAir) {
                double yVel = (pos.getY() / timeInAir)
                                - (speed.vyMetersPerSecond * movementMultiplierY);
                // SmartDashboard.putNumber("yVel", yVel);
                return yVel;
        }

        // returns the angle (wrt parralel to the target opening) to the target opening
        // from the robot
        // a value of pi/2, for example, means directly north (from a bird's eye view)
        // with the subwoofer north of the robot
        public static double calcRobotAngle(Pose2d pos, ChassisSpeeds speed, double flywheelSpeedMTS) {
                double timeInAir = calcTimeInAir(pos, speed, flywheelSpeedMTS);
                return Math.atan2(calcLaunchYVel(pos, speed, timeInAir),
                                calcLaunchXVel(pos, speed, timeInAir)) - Math.PI;

        }

        // returns the angle of the launcher required
        // 0 is parallel to the floor, pi/2 is vertically upwards
        public static double calcSubwooferLaunchAngle(Pose2d pos, ChassisSpeeds speed, double flywheelSpeedMTS) {
                pos = convert2dCoords(pos);
                speed = convertSpeed(pos, speed);
                double timeInAir = calcTimeInAir(pos, speed, flywheelSpeedMTS);
                double angleDiffRadians = (Math.PI
                                + (Math.atan2(calcLaunchVerticalVel(pos, speed, timeInAir),
                                                -Math.sqrt(Math.pow(calcLaunchXVel(pos, speed, timeInAir), 2) + Math
                                                                .pow(calcLaunchYVel(pos, speed, timeInAir), 2)))));
                double normalizedAngleDiff = angleDiffRadians
                                / (2 * Math.PI);
                // SmartDashboard.putNumber("targetAngleShoote", angleDiffRadians);
                // SmartDashboard.putNumber("PIVOT HEIGHT", encoderOffset
                // - normalizedAngleDiff);
                return encoderOffset
                                - normalizedAngleDiff;
        }

        public static void configHeightDif(double targetHeightDiff) {
                constTargetHeightDiff = targetHeightDiff;
                // SmartDashboard.putNumber("targetHeightDiff", constTargetHeightDiff);
        }

        public static double getHeightDif() {
                return constTargetHeightDiff;
        }

        // Calculates the time the note will spend in the air - linear aproximation
        // error margins depend on the distance from the amp
        // but errors are acceptably minimal via the approximation
        private static double calcTimeInAir(Pose2d pos, ChassisSpeeds speed, double flywheelSpeedMTS) {
                return Math.sqrt((constTargetHeightDiff * constTargetHeightDiff) + (pos.getX() * pos.getX())
                                + (pos.getY() * pos.getY()))
                                / (flywheelSpeedMTS * flywheelSpeedMultiplier);
        }

        private static double calcFerryDistance(Pose2d pos, Pose2d target) {
                return Math.sqrt(Math.pow(target.getX() - pos.getX(), 2) + Math.pow(target.getY() - pos.getY(), 2));
        }

        public static double calcFerryRotation(Pose2d pos, Pose2d target) {
                return Math.atan2(target.getY() - pos.getY(), target.getX() - pos.getX()) - Math.PI;
        }

        public static double calcFerryVelocity(Pose2d pos, Pose2d target) {
                // if ((calcFerryDistance(pos, target))<7.1) {
                // return 7.1;
                // }
                return ((calcFerryDistance(pos, target)) / 2.0 + 5.0);
        }

        public static double calcFerryLaunchAngle(Pose2d pos, Pose2d target) {
                // assumed transferred flywheel speed
                double v = calcFerryVelocity(pos, target);
                // distance from the robot to the target position
                double d = calcFerryDistance(pos, target);
                // launch angle
                double angle = encoderOffset - (Math.PI / 2.0 + 2.0 * Math.atan(
                                (5.0 * v * v / (49.0 * d)) -
                                                Math.sqrt(5.0) * Math.sqrt((5.0 * v * v * v * v) +
                                                                (49 * v * v * d)) / (49.0 * d)
                                                +
                                                (1.0 / 49.0) * Math.sqrt(
                                                                (50.0 * v * v * v * v) / (d * d) -
                                                                                (245.0 * v * v) / d +
                                                                                (4802.0 * Math.sqrt(5) * v * v)
                                                                                                / Math.sqrt((5.0 * v * v
                                                                                                                * v * v)
                                                                                                                + (49.0 * v * v * d))
                                                                                -
                                                                                (50.0 * Math.sqrt(5.0) * v * v * v * v
                                                                                                * v * v)
                                                                                                / (d * d * Math.sqrt(
                                                                                                                (5.0 * v * v * v * v)
                                                                                                                                + (49.0 * v * v * d)))
                                                                                -
                                                                                2401.0)))
                                / (2 * Math.PI);
                // SmartDashboard.putNumber("d", d);
                // SmartDashboard.putNumber("v", v);
                // SmartDashboard.putNumber("ahhhh", (Math.PI / 2.0 + 2.0 * Math.atan(
                // (5.0 * v * v / (49.0 * d)) -
                // Math.sqrt(5.0) * Math.sqrt((5.0 * v * v * v * v) +
                // (49 * v * v * d)) / (49.0 * d)
                // +
                // (1.0 / 49.0) * Math.sqrt(
                // (50.0 * v * v * v * v) / (d * d) -
                // (245.0 * v * v) / d +
                // (4802.0 * Math.sqrt(5) * v * v)
                // / Math.sqrt((5.0 * v * v
                // * v * v)
                // + (49.0 * v * v * d))
                // -
                // (50.0 * Math.sqrt(5.0) * v * v * v * v
                // * v * v)
                // / (d * d * Math.sqrt(
                // (5.0 * v * v * v * v)
                // + (49.0 * v * v * d)))
                // -
                // 2401.0)) // this last division is to
                // // normalize the angle
                // // difference
                // ) / (2 * Math.PI));

                return !Double.isNaN(angle) ? angle : encoderOffset - 0.07;
        }
}