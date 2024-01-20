package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterSpeeds;

public class ShooterFlywheel extends SubsystemBase{
    private static ShooterFlywheel shooterFlywheel;
    public static ShooterFlywheel getInstance() {
        if (shooterFlywheel == null) {
            shooterFlywheel = new ShooterFlywheel();
        }
        return shooterFlywheel;
    }
    
    private CANSparkFlex motorOne;
    private SparkPIDController motor1pid;

    private CANSparkFlex motorTwo;
    private SparkPIDController motor2pid;

    private double lastSpeed;

    public ShooterFlywheel(){
        motorOne = new CANSparkFlex(3, MotorType.kBrushless);
        motorTwo = new CANSparkFlex(12, MotorType.kBrushless);
        
        motor1pid = motorOne.getPIDController();
        motor1pid.setP(0);
        motor1pid.setFF(0.000155);

        motor2pid = motorTwo.getPIDController();
        motor2pid.setP(0);
        motor2pid.setFF(0.000155);

        motorTwo.follow(motorOne);
    }

    public void setFlywheelSpeed(double newSpeed){
        lastSpeed = newSpeed;
        motorOne.getPIDController().setReference(newSpeed, ControlType.kVelocity);
    }

    public void increaseFlywheelSpeed(double increment) {
        lastSpeed += increment;
        motorOne.getPIDController().setReference(lastSpeed, ControlType.kVelocity);
    }
    
    public void setFlywheelSpeed(ShooterSpeeds shooterSpeeds){
        setFlywheelSpeed(shooterSpeeds.flywheels);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("set speed", lastSpeed);
        SmartDashboard.putNumber("current speed", getFlywheelSpeed());
        SmartDashboard.putNumber("current speed 2", getAuxiluryFlywheelSpeed());

        SmartDashboard.putNumber("P", motor1pid.getP());
        SmartDashboard.putNumber("F", motor1pid.getFF());
    }
}