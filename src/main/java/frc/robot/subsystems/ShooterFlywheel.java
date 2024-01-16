package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
    private CANSparkFlex motorTwo;

    private double lastSpeed;

    public ShooterFlywheel(){
        motorOne = new CANSparkFlex(0, MotorType.kBrushless);
        motorTwo = new CANSparkFlex(0, MotorType.kBrushless);
        motorTwo.follow(motorOne);
    }

    public void setFlywheelSpeed(double newSpeed){
        lastSpeed = newSpeed;
        motorOne.getPIDController().setReference(newSpeed, ControlType.kVelocity);
    }
    
    public void setFlywheelSpeed(ShooterSpeeds shooterSpeeds){
        setFlywheelSpeed(shooterSpeeds.flywheels);
    }

    public double getLastTargetSpeed() {
       return lastSpeed; 
    }
}