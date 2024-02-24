package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterSpeeds;

public class ShooterTrack extends SubsystemBase {

    public static ShooterTrack shootingRetainer;
    private CANSparkFlex motor1;
    private DigitalInput sensor;

    /*
     * Things this needs to do:
     * 1. needs to be able to run motor when told
     * 2. needs to be able to check if the belt has a Note
     * 3. needs to be able to
     */

    public static ShooterTrack getInstance() {
        if (shootingRetainer == null) {
            shootingRetainer = new ShooterTrack();
        }
        return shootingRetainer;
    }

    public ShooterTrack() {
        motor1 = new CANSparkFlex(15, MotorType.kBrushless);
        motor1.restoreFactoryDefaults();
        motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
        motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
        motor1.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

        motor1.setCANTimeout(1000);

        motor1.getPIDController().setFeedbackDevice(motor1.getEncoder());
        motor1.setIdleMode(IdleMode.kBrake);
        motor1.setSmartCurrentLimit(30, 30);
        motor1.setInverted(false);

        sensor = new DigitalInput(0);
    }

    public void set(double percent) {
        motor1.set(percent);
    }

    public void set(ShooterSpeeds shooterSpeeds) {
        set(shooterSpeeds.track);
    }

    public boolean getSensor() {
        return sensor.get();
    }
}
