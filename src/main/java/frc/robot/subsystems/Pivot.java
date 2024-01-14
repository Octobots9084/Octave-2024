package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmPositions;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxEncoderType;
import frc.robot.util.SparkMax.SparkMaxSetup;
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
        SparkMaxConfig lead = new SparkMaxConfig(new SparkMaxStatusFrames(500, 20, 500, 500, 500, 20, 500), 1000, true, SparkMaxEncoderType.Absolute, IdleMode.kBrake, 30, 30, false, false, 1);
        SparkMaxSetup.setup(leadMotor, lead);
        SparkMaxSetup.setup(followMotor, follow);
        followMotor.follow(leadMotor);
    }
    
    public void setPosition(double target){
        target = MathUtil.clamp(target, minLimit, maxLimit);
        leadMotor.getPIDController().setReference(target, ControlType.kPosition);
        position = target;
    }

    public void setPosition(ArmPositions armPositions) {
        setPosition(armPositions);
    }

    public double getPosition(){
        return leadMotor.getEncoder().getPosition();
    }

    public double getLastTargetPosition(){
        return position;
    }

    public void setIdleMode(IdleMode idleMode){
        leadMotor.setIdleMode(idleMode);
    }


}


