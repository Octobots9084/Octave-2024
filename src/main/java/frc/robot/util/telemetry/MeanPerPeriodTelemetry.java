package frc.robot.util.telemetry;

/**
 * Telemetry class that reports the mean of the samples collected per period
 * (e.g. average vision confidence per second).
 */
public class MeanPerPeriodTelemetry extends BasePerPeriodTelemetry {
    private final double _defaultValueIfNoSamples;
    private double _sum;
    private int _count;

    /**
     * @param smartDashboardKey       the key (property name) that is sent to
     *                                SmartDashboard.
     * @param reportingPeriodSeconds  how often to update the value on
     *                                SmartDashboard
     * @param defaultValueIfNoSamples the value to report if no samples are recorded
     *                                at the time of
     *                                reporting
     */
    public MeanPerPeriodTelemetry(String smartDashboardKey, double reportingPeriodSecs,
            double defaultValueIfNoSamples) {
        super(smartDashboardKey, reportingPeriodSecs);
        _defaultValueIfNoSamples = defaultValueIfNoSamples;
    }

    @Override
    public String getTelemetryValue() {
        double mean = _defaultValueIfNoSamples;
        if (_count > 0) {
            mean = _sum / _count;
        }
        _sum = 0;
        _count = 0;

        return String.format("%.2f", mean);
    }

    /** Increments the internal counter with the given value (usually {@code 1}). */
    public void addNumber(double number) {
        _sum += number;
        ++_count;
    }
}
