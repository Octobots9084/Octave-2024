package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxEncoderType;
import frc.robot.util.SparkMax.SparkMaxSetup;
import frc.robot.util.SparkMax.SparkMaxStatusFrames;

public class elevator {
    public CANSparkMax motor1;
    public CANSparkMax motor2;
    //TODO:make this exist
    public boolean limSwitch;
    //TODO:get real value from CAD when possible
    private double gearing = 1;
    //TODO:placeholder values
    public double max = 10;
    public double min = 1;
    //TODO:placeholder
    private SparkMaxConfig elevateConfig = new SparkMaxConfig(new SparkMaxStatusFrames(300, 300, 10, 300, 300, 300, 300), 1, true, SparkMaxEncoderType.Relative, IdleMode.kBrake,
    30, 30, false, false, 50);
    private SparkMaxConfig followConfig = new SparkMaxConfig(new SparkMaxStatusFrames(300, 300, 10, 300, 300, 300, 300), 1, true, IdleMode.kBrake, 30, 30, false, motor1);
    
    public elevator (CANSparkMax motor1, CANSparkMax motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        SparkMaxSetup.setup(motor1, elevateConfig);
        SparkMaxSetup.setup(motor2, followConfig);
    }

    public double zero() {
        while(!limSwitch) {
            motor1.setVoltage(0.1);
        }
        motor1.stopMotor();

        return motor1.getEncoder().getPosition() / gearing;
    }

    public void setPosition(double pos) {
        if((max > pos) && (min < pos)) {
            motor1.getPIDController().setReference(gearing*pos, ControlType.kPosition);
        } 
    }

    public double getPosition() {
        return motor1.getEncoder().getPosition() / gearing;
    }

    public void goToMax() {
        motor1.getPIDController().setReference(gearing * max, ControlType.kPosition);
    }

    public void goToMin() {
        motor1.getPIDController().setReference(gearing * min, ControlType.kPosition);
    }
}