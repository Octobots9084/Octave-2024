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
        private static double constTargetHeightDiff = 1.4;
        // gravity
        private static double g = 9.8;
        // the final y velocity for the note to be moving at when it enters the target

        // X and Y positions of the subwoofer with regards to (0,0) on the robot's
        // Pose2d
        private static double subwooferXPos = 0;
        private static double subwooferYPos = 5.5;
        private static double encoderOffset = 0.597;
        private static double movementMultiplierX = 0;
        private static double movementMultiplierY = 0;
        private static double flywheelSpeedMultiplier = 0.9;

        // converts Pose2d coords into positions relative to the target
        public static Pose2d convert2dCoords(Pose2d pos) {
                if (Constants.isBlueAlliance) {
                        subwooferXPos = 0;
                        subwooferYPos = 5.5;
                } else {
                        subwooferXPos
                }
                SmartDashboard.putString("poseconvert",
                                new Pose2d(pos.getX() - subwooferXPos, pos.getY() - subwooferYPos, new Rotation2d())
                                                .toString());
                return new Pose2d(pos.getX() - subwooferXPos, pos.getY() - subwooferYPos, new Rotation2d());
        }

        // converts ChassisSpeeds to absolute rather than relative to the robot rotation
        public static ChassisSpeeds convertSpeed(Pose2d pos, ChassisSpeeds speed) {
                return new ChassisSpeeds(
                                -(pos.getRotation().getCos() * speed.vxMetersPerSecond)
                                                - (pos.getRotation().getSin() * speed.vyMetersPerSecond),
                                (pos.getRotation().getSin() * speed.vxMetersPerSecond)
                                                + (pos.getRotation().getCos() * speed.vyMetersPerSecond),
                                0);
        }

        // returns the vertical launch velocity of the note
        // for internal use only
        private static double calcLaunchVerticalVel(Pose2d pos, ChassisSpeeds speed,
                        double timeInAir) {
                double heightDelta = (g * Math.pow(timeInAir, 2)) / 2;
                double verticalVel = ((constTargetHeightDiff + heightDelta)
                                / timeInAir);
                SmartDashboard.putNumber("verticalVel", verticalVel);
                SmartDashboard.putNumber("heightDelta", heightDelta);
                return verticalVel;
        }

        // returns the horizontal (parallel to the subwoofer opening) launch velocity of
        // the note
        // for internal use only
        private static double calcLaunchXVel(Pose2d pos, ChassisSpeeds speed,
                        double timeInAir) {
                double xVel = (pos.getX() / timeInAir) - (speed.vxMetersPerSecond * movementMultiplierX);
                SmartDashboard.putNumber("xVel", xVel);
                return xVel;
        }

        // returns the "vertical" from above (perpendicular to the subwoofer opening)
        // launch velocity of the note
        // for internal use only
        private static double calcLaunchYVel(Pose2d pos, ChassisSpeeds speed,
                        double timeInAir) {
                double yVel = (pos.getY() / timeInAir)
                                - (speed.vyMetersPerSecond * movementMultiplierY);
                SmartDashboard.putNumber("yVel", yVel);
                return yVel;
        }

        // returns total speed the note should be launched at, in m/s
        public static double calcTotalLaunchVelocity(Pose2d pos, ChassisSpeeds speed, double flywheelSpeedMTS) {
                double timeInAir = Math.sqrt(constTargetHeightDiff * constTargetHeightDiff + pos.getX() * pos.getX())
                                / (flywheelSpeedMTS * flywheelSpeedMultiplier);
                return Math.sqrt(Math.pow(calcLaunchXVel(pos, speed, timeInAir), 2)
                                /*
                                 * + Math.pow(calcLaunchYVel(pos, speed), 2)
                                 */ + Math.pow(calcLaunchVerticalVel(pos, speed, timeInAir), 2));
        }

        // returns the angle (wrt parralel to the target opening) to the target opening
        // from the robot
        // a value of pi/2, for example, means directly north (from a bird's eye view)
        // with the subwoofer north of the robot
        public static double calcRobotAngle(Pose2d pos, ChassisSpeeds speed, double flywheelSpeedMTS) {
                double timeInAir = Math.sqrt(constTargetHeightDiff * constTargetHeightDiff + pos.getX() * pos.getX())
                                / (flywheelSpeedMTS * flywheelSpeedMultiplier);
                return Math.atan2(calcLaunchYVel(pos, speed, timeInAir),
                                calcLaunchXVel(pos, speed, timeInAir)) - Math.PI;

        }

        // returns the angle of the launcher required
        // 0 is parallel to the floor, pi/2 is vertically upwards
        public static double calcSubwooferLaunchAngle(Pose2d pos, ChassisSpeeds speed, double flywheelSpeedMTS) {
                double timeInAir = Math.sqrt(constTargetHeightDiff * constTargetHeightDiff + pos.getX() * pos.getX())
                                / flywheelSpeedMTS;
                pos = convert2dCoords(pos);
                speed = convertSpeed(pos, speed);
                SmartDashboard.putNumber("targetAngleShoote",
                                (Math.PI + (Math.atan2(calcLaunchVerticalVel(pos, speed, timeInAir),
                                                calcLaunchXVel(pos, speed, timeInAir)))));
                double angleDiffRadians = (Math.PI
                                + (Math.atan2(calcLaunchVerticalVel(pos, speed, timeInAir),
                                                calcLaunchXVel(pos, speed, timeInAir))));
                double normalizedAngleDiff = angleDiffRadians
                                / (2 * Math.PI);
                return encoderOffset
                                - normalizedAngleDiff;
        }

        public static void configHeightDif(double targetHeightDiff) {
                constTargetHeightDiff = targetHeightDiff;
                SmartDashboard.putNumber("targetHeightDiff", constTargetHeightDiff);
        }

        public static double getHeightDif() {
                return constTargetHeightDiff;
        }
}