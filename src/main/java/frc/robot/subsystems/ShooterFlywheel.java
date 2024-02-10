package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkMaxPIDController;
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
        motorOne = new CANSparkFlex(0, MotorType.kBrushless);
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
        motorOne.setIdleMode(IdleMode.kBrake);
        motorOne.setSmartCurrentLimit(10, 30);
        motorOne.setInverted(false);
        motorOne.getPIDController().setPositionPIDWrappingEnabled(false);
        motorOne.getPIDController().setP(0);
        motorOne.getPIDController().setI(0);
        motorOne.getPIDController().setD(0);
        motorOne.getPIDController().setFF(0);

        motorTwo = new CANSparkFlex(0, MotorType.kBrushless);
        motorTwo.restoreFactoryDefaults();
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motorOne.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
        motorTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

        motorTwo.setCANTimeout(1000);

        motorTwo.setIdleMode(IdleMode.kBrake);
        motorTwo.setSmartCurrentLimit(10, 30);
        motorTwo.setInverted(false);
        motorTwo.follow(motorOne);

        motorTwo = new CANSparkFlex(12, MotorType.kBrushless);

        motorOnepid = motorOne.getPIDController();

        motor2pid = motorTwo.getPIDController();

        // FIXME
        circumference = 1;
        // this needs to be made accurate later
        // it is the circumference of a launcher flywheel
    }

    public void setFlywheelSpeed(double newSpeed) {
        lastSpeed = newSpeed;
        motorOne.getPIDController().setReference(newSpeed, ControlType.kVelocity);
    }

    public void increaseFlywheelSpeed(double increment) {
        lastSpeed += increment;
        motorOne.getPIDController().setReference(lastSpeed, ControlType.kVelocity);
    }

    public void setFlywheelSpeed(ShooterSpeeds shooterSpeeds) {
        setFlywheelSpeed(shooterSpeeds.flywheels);
    }

    public void setFlyWheelSpeedMeters(double speed) {
        motorOne.getPIDController().setReference(lastSpeed / circumference, ControlType.kVelocity);
    }

    public double getFlywheelSpeedMeters() {
        return getFlywheelSpeed() * circumference;
    }

    public double getLastTargetSpeed() {
        return lastSpeed;
    }

    public double getFlywheelSpeed() {
        return motorOne.getEncoder().getVelocity();
    }

    public double getAuxiluryFlywheelSpeed() {
        return motorTwo.getEncoder().getVelocity();
    }
}