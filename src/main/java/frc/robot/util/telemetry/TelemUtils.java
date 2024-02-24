package frc.robot.util.telemetry;

/**
 * Utils for telemetry.
 */
public class TelemUtils {
    /** Generates a key name to use when sending camera-specific telemetry to SmartDashboard.
     * 
     * @param cameraName the name of the camera, e.g. "Inky", "Pinky", etc.
     * @param metricDesc very short and abbreviated description of the metric. Needs to be short so
     *  that it fits on SmartDashboard. e.g. "runs per s", "meas per s push", "meas per s pull", etc.
     * @return a concatenated string with the "Vision/" prefix. E.g."Vision/Inky - runs per s".
     */
    public static String getCamSDKey(String cameraName, String metricDesc) {
        return "Vision/" + cameraName + " - " + metricDesc;
    }
}
