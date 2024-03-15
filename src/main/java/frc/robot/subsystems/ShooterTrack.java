package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterSpeeds;

public class ShooterTrack extends SubsystemBase {

    public static ShooterTrack shootingRetainer;
    private TalonFX motor1;
    private DigitalInput sensor;
    public boolean pieceGood = false;
    public boolean currentlyShooting = false;

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

        motor1 = new TalonFX(15);

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
