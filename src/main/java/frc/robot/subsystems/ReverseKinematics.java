package frc.robot.subsystems;

// velocities are positive going towards the target and negative when moving away
// all units should be in meters, m/s, rad, etc.
// xPos is the distance from being in line with the amp
// yPos is distance from the amp (including the extended base part of it)

public class ReverseKinematics {
    // distance between launcher opening and the amp target
    private static double constTargetHeightDiff = 2.05;
    // gravity
    private static double g = -9.8;
    // the final y velocity for the note to be moving at when it enters the target
    private static double vf = 10;

    // returns the vertical launch velocity of the note
    // for internal use only
    public static double calcLaunchVerticalVel() {
        // System.out.println("time: " + (constTargetHeightDiff/Math.sqrt(Math.abs(vf - 2*constTargetHeightDiff*g))));
        return Math.sqrt(Math.abs((vf * vf) - (2*constTargetHeightDiff*g))); //Math.sqrt(constTargetHeightDiff * g * g);
    }

    // returns the horizontal (parallel to the amp opening) launch velocity of the note
    // for internal use only
    public static double calcLaunchXVel(double xPos, double xVel) {
        //System.out.println("launch x velocity: " + (xPos/(constTargetHeightDiff/calcLaunchVerticalVel()) - xVel));
        return xPos/(constTargetHeightDiff/calcLaunchVerticalVel()) - xVel;
    }

    // returns the "vertical" from above (perpendicular to the amp opening) launch velocity of the note
    // for internal use only
    public static double calcLaunchYVel(double yPos, double yVel) {
        //System.out.println("launch y velocity: " + (yPos/(constTargetHeightDiff/calcLaunchVerticalVel()) - yVel));
        return yPos/(constTargetHeightDiff/calcLaunchVerticalVel()) - yVel;
    }

    // returns total speed the note should be launched at, in m/s
    public static double calcTotalLaunchVelocity(double xPos, double yPos, double xVel, double yVel) {
        return Math.sqrt(Math.pow(calcLaunchXVel(xPos, xVel), 2) + Math.pow(calcLaunchYVel(yPos, yVel), 2) + Math.pow(calcLaunchVerticalVel(), 2));
    }

    // returns the angle (wrt parralel to the target opening) to the target opening from the robot
    // a value of pi/2, for example, means directly north (from a bird's eye view) with the amp north of the robot
    public static double calcRobotAngle(double xPos, double yPos, double xVel, double yVel) {
        return xPos >= 0 ? Math.atan(calcLaunchYVel(yPos, yVel)/calcLaunchXVel(xPos, xVel)) : (Math.PI / 2.0) - Math.atan(calcLaunchYVel(yPos, yVel)/calcLaunchXVel(xPos, xVel));
    }

    // returns the angle of the launcher required
    // 0 is parallel to the floor, pi/2 is vertically upwards
    public static double calcAmpLaunchAngle(double xPos, double yPos, double xVel, double yVel) {
        return Math.atan(calcLaunchVerticalVel()/Math.sqrt(Math.pow(calcLaunchXVel(xPos, xVel), 2) + Math.pow(calcLaunchYVel(yPos, yVel), 2)));
    }
}
