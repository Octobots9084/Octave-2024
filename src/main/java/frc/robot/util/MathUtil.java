package frc.robot.util;

public class MathUtil {
    /**
     * <p>
     * Wraps a circular angle value to within one circle.
     * </p>
     *
     * <p>
     * Customizable number of angle units per circle. Angle is
     * equivalent and wrapped to the positive [0, fullCircle] range.
     * </p>
     *
     * @param angle      the raw angle units to wrap.
     * @param fullCircle the number of angle units to be one circle.
     * @return the wrapped angle in corresponding angle units.
     */
    public static double wrapToCircle(double angle, double fullCircle) {
        angle %= fullCircle;
        return angle < 0 ? fullCircle + angle : angle;
    }

    /**
     * Checks if a value is within a certain tolerance of a target. Directions
     * irrelevant.
     *
     * @param value     the current value for which to check.
     * @param target    the target to check the value against.
     * @param tolerance the tolerance (positive and negative directions)
     *                  around the target that is acceptable error
     *                  for the value to be "within tolerance".
     * @return if the value is within tolerance of the target.
     */
    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) < tolerance;
    }

    public static double clampAngle(double angle) {
        double high = Math.PI;
        angle = MathUtil.wrapToCircle(angle, 2 * Math.PI);
        if (angle > high) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * <p>
     * Fits value between -1 and 1 but eliminates noise between the deadband.
     * </p>
     *
     * <p>
     * Generally used for eliminating noise in joystick readings by setting the
     * output to zero when within a certain deadband amount of zero. Non-joystick
     * values are also limited between -1 and 1 so as to use in a motor set.
     * </p>
     *
     * @param val      the value to fit inside the valid range and outside the
     *                 deadband.
     * @param deadband the amount of tolerance around zero in which
     *                 values are set to zero.
     * @return the value fitted to the range.
     *
     * @since 0.1.0
     */
    public static double fitDeadband(double val, double deadband) {
        if (Math.abs(val) >= deadband) {
            if (val > 0) {
                if (val >= 1) {
                    return 1;
                } else {
                    return (val - deadband) / (1 - deadband);
                }
            } else if (val < 0) {
                if (val <= -1) {
                    return -1;
                } else {
                    return (val + deadband) / (1 - deadband);
                }
            }
        }
        return 0;
    }

    /**
     * Flips an angle over the y axis
     * 
     * @param angle The angle in degrees
     * @return The flipped angle
     */
    public static double flipAngleOverYAxis(double angle) {
        double radians = Math.toRadians(angle);
        double sin = Math.sin(radians);
        double cos = Math.cos(radians);
        double mirroredCos = -cos;
        double mirroredRadians = Math.atan2(sin, mirroredCos);
        double mirroredAngle = Math.toDegrees(mirroredRadians);
        return mirroredAngle;
    }

    public static boolean isPointInTriangle(double x1, double y1, double x2, double y2, double x3, double y3, double xp,
            double yp) {
        // Calculate the area of the triangle ABC
        double areaABC = Math.abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);

        // Calculate the area of triangles PAB, PBC, and PCA using the same formula
        double areaPAB = Math.abs((xp * (y2 - y3) + x2 * (y3 - yp) + x3 * (yp - y2)) / 2.0);
        double areaPBC = Math.abs((xp * (y1 - y3) + x1 * (y3 - yp) + x3 * (yp - y1)) / 2.0);
        double areaPCA = Math.abs((xp * (y2 - y1) + x2 * (y1 - yp) + x1 * (yp - y2)) / 2.0);

        // If the sum of the areas of PAB, PBC and PCA is equal to the area of ABC, then the point is inside the triangle
        return (Math.abs(areaABC - (areaPAB + areaPBC + areaPCA)) < 0.0001);
    }

}
