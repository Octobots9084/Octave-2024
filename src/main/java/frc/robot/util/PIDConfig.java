package frc.robot.util;

/**
 * <p>
 * Stores a PIDF configuration through loop gain constants.
 * </p>
 *
 * <p>
 * Terms may be accessed or changed through standard getters/setters.
 * Contains helper method to apply terms to a given CTRE motor controller.
 * Should be used in all places where PIDF loops are needed.
 * </p>
 *
 * @since 0.1.0
 */
public class PIDConfig {
    protected double kP;
    protected double kI;
    protected double kD;
    protected double kF;
    protected double tolerance;
    protected Double iZone;

    /**
     * Constructs a PID configuration with the specified loop gain constants.
     *
     * @param kP    proportional gain; output proportional to current error.
     * @param kI    integral gain; output based on accumulated error to exponentiate
     *              kP.
     * @param kD    derivative gain; typically used for damping.
     * @param kF    feed forward constant.
     * @param iZone The range allowed for kI (Needs to be a double object so it can
     *              be null)
     *
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD, double kF, Double iZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iZone = iZone;
    }

    /**
     * <p>
     * Constructs a PID configuration with the specified loop gain constants.
     * Has a default range value of [-1, 1] or full range.
     * </p>
     *
     * <p>
     * Overload for {@link #PIDConfig(double, double, double, double, Double)}.
     * </p>
     *
     * @see #PIDConfig(double, double, double, double, double)
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, null);
    }

    /**
     * <p>
     * Constructs a PID configuration with the specified loop gain constants.
     * Has a default range value of [-1, 1] or full range and no feed forward.
     * </p>
     *
     * <p>
     * Overload for {@link #PIDConfig(double, double, double, double, double)}.
     * </p>
     *
     * @see #PIDConfig(double, double, double, double, double)
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, null);
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public double getTolerance() {
        return tolerance;
    }

    public Double getIZone() {
        return iZone;
    }

    public PIDConfig setP(double kP) {
        this.kP = kP;
        return this;
    }

    public PIDConfig setI(double kI) {
        this.kI = kI;
        return this;
    }

    public PIDConfig setD(double kD) {
        this.kD = kD;
        return this;
    }

    public PIDConfig setF(double kF) {
        this.kF = kF;
        return this;
    }

    public PIDConfig setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public PIDConfig setIZone(double iZone) {
        this.iZone = iZone;
        return this;
    }
}
