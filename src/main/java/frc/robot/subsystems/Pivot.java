package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxStatusFrames;

public class Pivot extends SubsystemBase{
    public static Pivot pivot;
    public static Pivot getInstance(){
        if(pivot == null){
            pivot = new Pivot();
        }
        return pivot;
    }
    
    private static double minLimit = 0, maxLimit = 0;
    private CANSparkMax leadMotor, followMotor;
    private double position;


    public Pivot(){
        leadMotor = new CANSparkMax(0, MotorType.kBrushless);
        followMotor = new CANSparkMax(0, MotorType.kBrushless);
        SparkMaxConfig follow = new SparkMaxConfig(new SparkMaxStatusFrames(500, 20, 500, 500, 500, 20, 500), 1000, true, IdleMode.kBrake, 30, 30, false, leadMotor);
        SparkMaxConfig lead = new SparkMaxConfig(new SparkMaxStatusFrames(500, 20, 500, 500, 500, 20, 500), 1000, true, IdleMode.kBrake, 30, 30, false, followMotor);
    }
    
    public void setLocation(double target){
        target = MathUtil.clamp(target, minLimit, maxLimit);
        leadMotor.getPIDController().setReference(target, ControlType.kPosition);
    }

    public double getPosition(){
        return position;
    }

    public void setPosition(double newPosition){
        position = newPosition;
    }
}


