package frc.robot.subsystems;

// note: only works on the left side of the target

public class ReverseKinematics {
    // all units should be in meters, m/s, rad, etc.

    // distance between launcher opening and target height
    private static double constTargetHeightDiff = 13;
    // gravity
    private static double g = -9.8;

    public static double calcLaunchVerticalVel() {
        return -g*Math.sqrt(Math.abs(2*constTargetHeightDiff/g)); //Math.sqrt(constTargetHeightDiff * g * g);
    }

    // velocities are positive going towards the target and negative when moving away

    public static double calcLaunchXVel(double xPos, double xVel) {
        System.out.println("launch x velocity: " + (xPos/(constTargetHeightDiff/calcLaunchVerticalVel()) - xVel));
        return xPos/(2*constTargetHeightDiff/calcLaunchVerticalVel()) - xVel;
    }

    public static double calcLaunchYVel(double yPos, double yVel) {
        System.out.println("launch y velocity: " + (yPos/(constTargetHeightDiff/calcLaunchVerticalVel()) - yVel));
        return yPos/(2*constTargetHeightDiff/calcLaunchVerticalVel()) - yVel;
    }

    public static double calcTotalLaunchVelocity(double xPos, double yPos, double xVel, double yVel) {
        return Math.sqrt(Math.pow(calcLaunchXVel(xPos, xVel), 2) + Math.pow(calcLaunchYVel(yPos, yVel), 2) + Math.pow(calcLaunchVerticalVel(), 2));
    }

    // returns the angle (wrt parralel to the target opening) to the target opening from the robot
    public static double calcRobotAngle(double xPos, double yPos, double xVel, double yVel) {
        return Math.atan(calcLaunchYVel(yPos, yVel)/calcLaunchXVel(xPos, xVel));
    }

    // xPos is the distance from being in line with the amp
    // yPos is distance from the amp (including the extended base part of it)
    public static double calcAmpLaunchAngle(double xPos, double yPos, double xVel, double yVel) {
        return Math.atan(calcLaunchVerticalVel()/Math.sqrt(Math.pow(calcLaunchXVel(xPos, xVel), 2) + Math.pow(calcLaunchYVel(yPos, yVel), 2)));
    }
}
