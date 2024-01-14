package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.constants.ShooterSpeeds;

public class Flywheel {
    private static Flywheel flywheel;
    private static CANSparkFlex motorOne;
    private static CANSparkFlex motorTwo;

    private static double lastSpeed;

    public Flywheel(){
        motorOne = new CANSparkFlex(0, MotorType.kBrushless);
        motorTwo = new CANSparkFlex(0, MotorType.kBrushless);
        motorTwo.follow(motorOne);
    }


    public static Flywheel getInstance(){
        if(flywheel == null){
            flywheel = new Flywheel();
        }
        return flywheel;
    }

    public static void setFlywheelSpeed(double newSpeed){
        lastSpeed = newSpeed;
        motorOne.getPIDController().setReference(newSpeed, ControlType.kVelocity);
    }
    
    public static void setFlywheelSpeed(ShooterSpeeds shooterSpeeds){
        setFlywheelSpeed(shooterSpeeds.flywheels);
    }

    public double getLastTargetSpeed() {
       return lastSpeed; 
    }
}