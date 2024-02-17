package frc.robot.util.telemetry;

/**
 * Telemetry class that reports the number of samples collected per period (e.g.
 * frames per second, number of vision measurements per half second, etc.)
 */
public class CountPerPeriodTelemetry extends BasePerPeriodTelemetry {
    private int _count;

    /**
     * @param smartDashboardKey      the key (property name) that is sent to
     *                               SmartDashboard.
     * @param reportingPeriodSeconds how often to update the value on SmartDashboard
     */
    public CountPerPeriodTelemetry(String smartDashboardKey, double reportingPeriodSecs) {
        super(smartDashboardKey, reportingPeriodSecs);
    }

    @Override
    public String getTelemetryValue() {
        final String value = Integer.toString(_count);
        _count = 0;
        return value;
    }

    /** Increments the internal counter with the given value (usually {@code 1}). */
    public void incCount(int increment) {
        _count += increment;
    }

}
