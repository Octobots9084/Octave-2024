package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxEncoderType;
import frc.robot.util.SparkMax.SparkMaxStatusFrames;

public class Intake extends SubsystemBase{
    private static Intake intake;
    public static Intake getInstance(){
        if (intake == null){
            intake = new Intake();
        }
        return intake;
    }
    CANSparkMax motor1;
    public Intake() {
        motor1 = new CANSparkMax(0, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig(new SparkMaxStatusFrames(500, 20, 500, 20, 500, 500, 500), 1000, true, SparkMaxEncoderType.Relative, IdleMode.kBrake, 30, 30, false, false, 1);
        
    }
}
