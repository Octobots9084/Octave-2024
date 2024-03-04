package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterSpeeds;

public class ShooterFlywheel extends SubsystemBase {
    private static ShooterFlywheel shooterFlywheel;

    public static ShooterFlywheel getInstance() {
        if (shooterFlywheel == null) {
            shooterFlywheel = new ShooterFlywheel();
        }
        return shooterFlywheel;
    }

    private CANSparkFlex motorOne;
    private SparkPIDController motorOnepid;

    private CANSparkFlex motorTwo;
    private SparkPIDController motor2pid;

    private double lastSpeed;
    private double circumference;

    public ShooterFlywheel() {
        motorOne = new CANSparkFlex(16, MotorType.kBrushless);
        motorOne.restoreFactoryDefaults();
        motorOne.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        motorOne.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motorOne.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        motorOne.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
        motorOne.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        motorOne.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
        motorOne.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

        motorOne.setCANTimeout(1000);

        motorOne.getPIDController().setFeedbackDevice(motorOne.getEncoder());
        motorOne.setIdleMode(IdleMode.kCoast);
        motorOne.setSmartCurrentLimit(40, 40);
        motorOne.setInverted(false);
        motorOne.getPIDController().setP(0.0015);
        motorOne.getPIDController().setI(0);
        motorOne.getPIDController().setD(0);
        motorOne.getPIDController().setFF(0.000185);
        motorOne.enableVoltageCompensation(10);
        // motorOne.getPIDController().setIZone(1);

        motorTwo = new CANSparkFlex(17, MotorType.kBrushless);
        motorTwo.restoreFactoryDefaults();
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

        motorTwo.setCANTimeout(1000);

        motorTwo.getPIDController().setFeedbackDevice(motorTwo.getEncoder());
        motorTwo.setIdleMode(IdleMode.kCoast);
        motorTwo.setSmartCurrentLimit(40, 40);
        motorTwo.setInverted(false);
        motorTwo.getPIDController().setP(0.0015);
        motorTwo.getPIDController().setI(0.0);
        motorTwo.getPIDController().setD(0);
        motorTwo.getPIDController().setFF(0.000183);
        motorTwo.enableVoltageCompensation(10);
        // motorTwo.getPIDController().setIZone(1);

        motorOnepid = motorOne.getPIDController();

        motor2pid = motorTwo.getPIDController();

        // FIXME
        circumference = 2 * Math.PI * 0.05;
        // this needs to be made accurate later
        // it is the circumference of a launcher flywheel
    }

    public void setFlywheelSpeed(double newSpeed) {
        lastSpeed = newSpeed;
        SmartDashboard.putNumber("newSpeed", newSpeed);
        motorOne.getPIDController().setReference(newSpeed, ControlType.kVelocity);
        motorTwo.getPIDController().setReference(-newSpeed, ControlType.kVelocity);
    }

    public void increaseFlywheelSpeed(double increment) {
        lastSpeed += increment;
        motorOne.getPIDController().setReference(lastSpeed, ControlType.kVelocity);
    }

    public void setFlywheelActive(boolean setFlywheelActive) {
        if (!setFlywheelActive) {
            motorOne.getPIDController().setReference(0, ControlType.kVoltage);
            motorTwo.getPIDController().setReference(0, ControlType.kVoltage);
        }
    }

    public void setFlywheelSpeed(ShooterSpeeds shooterSpeeds) {
        setFlywheelSpeed(shooterSpeeds.flywheels);
    }

    public void setFlyWheelSpeedMeters(double speed) {
        motorOne.getPIDController().setReference(60 * speed / circumference, ControlType.kVelocity);
        motorTwo.getPIDController().setReference(-60 * speed / circumference, ControlType.kVelocity);
    }

    public double getFlywheelSpeedMeters() {
        return (getFlywheelSpeed() * circumference) / 60;
    }

    public double getLastTargetSpeed() {
        return lastSpeed;
    }

    public double getFlywheelSpeed() {
        return motorOne.getEncoder().getVelocity();
    }

    public double getLeftFlywheelSpeed() {
        return motorOne.getEncoder().getVelocity();
    }

    public double getRightFlywheelSpeed() {
        return motorTwo.getEncoder().getVelocity();
    }

    public double getAuxiluryFlywheelSpeed() {
        return motorTwo.getEncoder().getVelocity();
    }

    public double getAuxiluryFlywheelSpeedMeters() {
        return (getAuxiluryFlywheelSpeed() * circumference) / 60;
    }

    public void setFlywheelPercent(double percent) {
        motorOne.set(-percent);
        motorTwo.set(percent);

    }

    public void setFlywheelVoltage(double voltage) {
        motorOne.setVoltage(-voltage);
        motorTwo.setVoltage(voltage);
    }

    public void setFlywheelsCurrentFast() {
        motorOne.setSmartCurrentLimit(100, 100);
    }

    public void setFlywheelsCurrentNormal() {
        motorOne.setSmartCurrentLimit(40, 40);
    }
}