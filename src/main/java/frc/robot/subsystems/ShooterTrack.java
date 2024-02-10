package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterSpeeds;

public class ShooterTrack extends SubsystemBase {

    public static ShooterTrack shootingRetainer;
    private CANSparkFlex motor1;

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
        motor1 = new CANSparkFlex(0, MotorType.kBrushless);
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
        motor1.setSmartCurrentLimit(10, 30);
        motor1.setInverted(false);
        motor1.getPIDController().setPositionPIDWrappingEnabled(false);
        motor1.getPIDController().setP(0);
        motor1.getPIDController().setI(0);
        motor1.getPIDController().setD(0);
    }

    public void setSpeed(double speed) {
        motor1.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    public void setSpeed(ShooterSpeeds shooterSpeeds) {
        setSpeed(shooterSpeeds.track);
    }

    public boolean getSensor() {
        return true;
    }
}
