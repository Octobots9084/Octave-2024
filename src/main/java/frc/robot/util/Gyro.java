package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class Gyro implements Sendable {
    private AHRS navX;
    private boolean simulated;

    // Degrees (ccw positive)
    int dev;
    SimDouble simulatedAngleDegrees;
    private double simulatedAngleAdjustment = 0;
    private double simulatedRotationSpeed = 0; // Radians per second
    private double lastTimeSimulatedRotationUpdated = 0;

    /**
     * Creates a gyro wrapper class for a navX connected to a robotrio
     * 
     * @param simulated    Whether the gyro is simulated
     * @param initialAngle The starting angle the robot starts at in degrees (ccw
     *                     positive)
     */
    public Gyro(boolean simulated, double initialAngle) {
        this.navX = new AHRS(SPI.Port.kMXP);
        this.simulated = simulated;
        if (simulated) {
            dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            simulatedAngleDegrees = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
            simulatedAngleDegrees.set(initialAngle);
        }

    }

    /**
     * Gets the continuous angle of the gyro
     * 
     * @return The continuous angle of the gyro in degrees (It goes beyond 1 full
     *         rotation) (ccw positive)
     */
    public double getContinuousAngleDegrees() {
        if (simulated) {
            final double timestampNow = Timer.getFPGATimestamp();

            simulatedAngleDegrees.set(simulatedAngleDegrees.get() + simulatedRotationSpeed
                    * (timestampNow - lastTimeSimulatedRotationUpdated));
            lastTimeSimulatedRotationUpdated = timestampNow;
            return simulatedAngleDegrees.get() + simulatedAngleAdjustment;
        } else {
            return -this.navX.getAngle();
        }
    }

    /**
     * Gets the wrapped angle of the gyro
     * 
     * @return The wrapped angle of the gyro in degrees (It is limited between 0 and
     *         360) (ccw positive)
     */
    public double getWrappedAngleDegrees() {
        return MathUtil.wrapToCircle(getContinuousAngleDegrees(), 360);
    }

    /**
     * Gets the unwrapped angle of the gyro as a rotation 2D
     * 
     * @return The unwrapped angle as a rotation 2D (ccw positive)
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(getContinuousAngleDegrees()));

    }

    /**
     * Gets the wrapped angle of the gyro
     * 
     * @return The wrapped angle as a rotation 2D (ccw positive)
     */
    public Rotation2d getWrappedAngleRotation2D() {
        return new Rotation2d(Math.toRadians(getWrappedAngleDegrees()));

    }

    /**
     * Sets the angle adjustment of the gyro (ccw positive)
     * 
     * @param angleAdjustment
     */
    public void setAngleAdjustment(double angleAdjustment) {
        if (simulated) {
            simulatedAngleAdjustment = angleAdjustment;
        } else {
            navX.setAngleAdjustment(angleAdjustment);
        }
    }

    /**
     * Sets the simulated rotation speed of the gyro
     * 
     * @param rotationSpeed The simulated rotation speed of the gyro (ccw positive)
     */
    public void setSimulatedRotationSpeed(double rotationSpeed) {
        final double timestampNow = Timer.getFPGATimestamp();

        simulatedAngleDegrees.set(simulatedAngleDegrees.get()
                + simulatedRotationSpeed * (timestampNow - lastTimeSimulatedRotationUpdated));
        lastTimeSimulatedRotationUpdated = timestampNow;
        this.simulatedRotationSpeed = rotationSpeed;
    }

    /**
     * Gets the angle adjustment of the gyro
     * 
     * @return The current angle adjustment of the gyro (ccw positive)
     */
    public double getAngleAdjustment() {
        if (simulated) {
            return simulatedAngleAdjustment;
        } else {
            return navX.getAngleAdjustment();
        }
    }

    /**
     * Resets the gyro yaw to 0
     */
    public void reset() {
        if (simulated) {
            simulatedAngleDegrees.set(0);

        } else {
            navX.reset();
        }

    }

    /**
     * Sets the simulated angle of the gyro (degrees ccw positive)
     * 
     * @param simulatedAngle
     */
    public void setSimulatedAngle(double simulatedAngle) {
        this.simulatedAngleDegrees.set(simulatedAngle);
    }

    /**
     * Sets the sendable properties
     * 
     * @param builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Yaw", () -> getContinuousAngleDegrees(), null);

    }

    public double getPitch() {
        return navX.getPitch();
    }
}
