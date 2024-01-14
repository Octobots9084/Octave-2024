package frc.robot.subsystems;

public class ReverseKinematics {
    // all units should be in meters, m/s, rad, etc.

    // distance between launcher opening and target height
    private static double constTargetHeightDiff = 1;
    // gravity
    private static double g = -9.8;

    public static double calcLaunchVerticalVel() {
        return Math.sqrt(constTargetHeightDiff * g * g);
    }

    // velocities are positive going towards the target and negative when moving away

    public static double calcLaunchXVel(double xPos, double xVel) {
        return xPos/(constTargetHeightDiff*calcLaunchVerticalVel()) - xVel;
    }

    public static double calcLaunchYVel(double yPos, double yVel) {
        return yPos/(constTargetHeightDiff*calcLaunchVerticalVel()) - yVel;
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
        return Math.atan(Math.sqrt(Math.pow(calcLaunchXVel(xPos, xVel), 2) + Math.pow(calcLaunchYVel(yPos, yVel), 2))/calcLaunchVerticalVel());
    }
}
