package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// velocities are positive going towards the target and negative when moving away
// all units should be in meters, m/s, rad, etc.
// xPos is the distance from being in line with the subwoofer
// yPos is distance from the subwoofer (including the extended base part of it)

public class ReverseKinematics {
    // distance between launcher opening and the subwoofer target
    private static double constTargetHeightDiff = 2.05;
    // gravity
    private static double g = -9.8;
    // the final y velocity for the note to be moving at when it enters the target
    private static double vf = 10;

    // X and Y positions of the subwoofer with regards to (0,0) on the robot's Pose2d
    private static double subwooferXPos = 0;
    private static double subwooferYPos = 5.5;
    private static double encoderOffset = 0.602;

    // converts Pose2d coords into positions relative to the target
    public static Pose2d convert2dCoords(Pose2d pos) {
        SmartDashboard.putString("poseconvert", new Pose2d(pos.getX() - subwooferXPos, subwooferYPos - pos.getY(), new Rotation2d()).toString());
        return new Pose2d(pos.getX() - subwooferXPos, pos.getY() - subwooferYPos, new Rotation2d());
    }

    // converts ChassisSpeeds to absolute rather than relative to the robot rotation
    public static ChassisSpeeds convertSpeed(Pose2d pos, ChassisSpeeds speed) {
        return new ChassisSpeeds(-(pos.getRotation().getCos()*speed.vxMetersPerSecond) - (pos.getRotation().getSin()*speed.vyMetersPerSecond), (pos.getRotation().getSin()*speed.vxMetersPerSecond) + (pos.getRotation().getCos()*speed.vyMetersPerSecond), 0);
    }

    // returns the vertical launch velocity of the note
    // for internal use only
    private static double calcLaunchVerticalVel() {
        // System.out.println("time: " + (constTargetHeightDiff/Math.sqrt(Math.abs(vf - 2*constTargetHeightDiff*g))));
        return Math.sqrt(Math.abs((vf * vf) - (2*constTargetHeightDiff*g))); //Math.sqrt(constTargetHeightDiff * g * g);
    }

    // returns the horizontal (parallel to the subwoofer opening) launch velocity of the note
    // for internal use only
    private static double calcLaunchXVel(Pose2d pos, ChassisSpeeds speed) {
        //System.out.println("launch x velocity: " + (xPos/(constTargetHeightDiff/calcLaunchVerticalVel()) - xVel));
        return pos.getX()/(constTargetHeightDiff/calcLaunchVerticalVel()) + speed.vxMetersPerSecond;
    }

    // returns the "vertical" from above (perpendicular to the subwoofer opening) launch velocity of the note
    // for internal use only
    private static double calcLaunchYVel(Pose2d pos, ChassisSpeeds speed) {
        //System.out.println("launch y velocity: " + (yPos/(constTargetHeightDiff/calcLaunchVerticalVel()) - yVel));
        return pos.getY()/(constTargetHeightDiff/calcLaunchVerticalVel()) + speed.vyMetersPerSecond;
    }

    // returns total speed the note should be launched at, in m/s
    public static double calcTotalLaunchVelocity(Pose2d pos, ChassisSpeeds speed) {
        return Math.sqrt(Math.pow(calcLaunchXVel(pos, speed), 2) + Math.pow(calcLaunchYVel(pos, speed), 2) + Math.pow(calcLaunchVerticalVel(), 2));
    }

    // returns the angle (wrt parralel to the target opening) to the target opening from the robot
    // a value of pi/2, for example, means directly north (from a bird's eye view) with the subwoofer north of the robot
    public static double calcRobotAngle(Pose2d pos, ChassisSpeeds speed) {
        return Math.atan2(calcLaunchYVel(pos, speed), calcLaunchXVel(pos, speed));
        //return pos.getX() >= 0 ? Math.atan(calcLaunchYVel(pos, speed)/calcLaunchXVel(pos, speed)) : (Math.PI / 2.0) - Math.atan(calcLaunchYVel(pos, speed)/calcLaunchXVel(pos, speed));
    }

    // returns the angle of the launcher required
    // 0 is parallel to the floor, pi/2 is vertically upwards
    public static double calcSubwooferLaunchAngle(Pose2d pos, ChassisSpeeds speed) {
        pos = convert2dCoords(pos);
        speed = convertSpeed(pos, speed);
        return encoderOffset - Math.atan2(calcLaunchVerticalVel(),calcLaunchXVel(pos, speed))/(2*Math.PI);
    }
}