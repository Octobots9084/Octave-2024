package frc.robot.util.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

/**
 * Base abstract class for reporting "per-period" telemetry on SmartDashboard.
 * Examples of per-period telemetry include: frames per second, measurements per
 * half second, average vision confidence per second, average vision ambiguity
 * per half second, etc.
 *
 * To add a specific telemetry, add a class that inherits this class. This new
 * class should implement the custom statistics / calculation that needs to be
 * reported, along with the actual string representation to be sent to
 * SmartDashboard. This is done in the {@link #getTelemetryValue()} method,
 * which must be implemented by the inheriting class.
 *
 * This base class provides only the per-period (timing) logic and the sending
 * of the data to SmartDashboard. The main method is {@link #periodic()} and
 * must to be called periodically and often enough (e.g. every 20 to 50 ms) to
 * ensure the telemetry is sent at the correct period.
 */
public abstract class BasePerPeriodTelemetry {
    private final String _smartDashboardKey;
    private final double _reportingPeriodSecs;
    private double _lastReportingTimestampSecs;

    /**
     * @param smartDashboardKey      the key (property name) that is sent to
     *                               SmartDashboard.
     * @param reportingPeriodSeconds how often to update the value on SmartDashboard
     */
    public BasePerPeriodTelemetry(String smartDashboardKey, double reportingPeriodSecs) {
        _smartDashboardKey = smartDashboardKey;
        _reportingPeriodSecs = reportingPeriodSecs;
        _lastReportingTimestampSecs = Timer.getFPGATimestamp();
    }

    /**
     * Implement this method to return the telemetry value to be sent to
     * SmartDashboard. For example, if you are implementing a "frames per second"
     * telemetry, this method should return the number of frames since the last
     * call and reset the frames counter to {@code 0}.
     */
    public abstract String getTelemetryValue();

    /**
     * Must be called periodically and often enough (e.g. every 20 to 50 ms). This
     * method will determine if it is time to send the telemetry to SmartDashboard.
     * In this case, the method will call the abstract {@link #getTelemetryValue()}
     * method and send its return value to SmartDashboard.
     */
    public void periodic() {
        final double currTimestampSecs = Timer.getFPGATimestamp();
        if (currTimestampSecs - _lastReportingTimestampSecs >= _reportingPeriodSecs) {
            _lastReportingTimestampSecs = currTimestampSecs;

            SmartDashboard.putString(_smartDashboardKey, getTelemetryValue());
        }
    }
}
