package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import frc.robot.constants.IntakeSpeeds;
import frc.robot.util.PIDConfig;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxEncoderType;
import frc.robot.util.SparkMax.SparkMaxSetup;
import frc.robot.util.SparkMax.SparkMaxStatusFrames;

public class IntakeTrack extends SubsystemBase{

    //Written by Olorin//

    public static IntakeTrack intakeRetainer;
    private CANSparkMax motor1;
    private SparkMaxConfig motor1Config;
    private boolean retainingNote;
    
    /* Things this needs to do:
    1. needs to be able to run motor when told
    2. needs to be able to check if the belt has a Note
    3. needs to be able to
     */

    public static IntakeTrack getInstance(){
        if (intakeRetainer == null){
            intakeRetainer = new IntakeTrack();
        }
        return intakeRetainer;
    }

    public IntakeTrack(){
        motor1 = new CANSparkMax(0, MotorType.kBrushless);
        motor1Config = new SparkMaxConfig(new SparkMaxStatusFrames(
            500,
            10,
            10,
            50,
            500,
            500,
            500),
            1000,
            true,
            SparkMaxEncoderType.Relative,
            IdleMode.kBrake,
            10,
            30,
            false,
            false,
            2048,
            false,
            new PIDConfig(5, 0, 0, 0));
        SparkMaxSetup.setup(motor1, motor1Config);
    }

    public boolean checkIntake(){
        return true;
    }
    
    public void setSpeed(double speed){
        motor1.set(speed);
    }

    public void setSpeed(IntakeSpeeds intakeSpeeds) {
        setSpeed(intakeSpeeds.track);
    }

    public boolean isRetainingNote() {
        return retainingNote;
    }
}
