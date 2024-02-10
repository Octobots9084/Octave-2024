package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbPositions;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxEncoderType;
import frc.robot.util.SparkMax.SparkMaxSetup;
import frc.robot.util.SparkMax.SparkMaxStatusFrames;

public class Climb extends SubsystemBase {
    private static Climb climb;

    public static Climb getInstance() {
        if (climb == null) {
            climb = new Climb();
        }
        return climb;
    }

    CANSparkMax motor1, motor2;
    public boolean limSwitch;
    private double gearing = 1;

    public Climb() {
        motor1 = new CANSparkMax(0, MotorType.kBrushless);
        motor2 = new CANSparkMax(0, MotorType.kBrushless);
        SparkMaxConfig leadConfig = new SparkMaxConfig(
                new SparkMaxStatusFrames(
                        20,
                        20,
                        20,
                        500,
                        500,
                        500,
                        500),
                1000,
                true,
                SparkMaxEncoderType.Relative,
                IdleMode.kBrake,
                30,
                30,
                false,
                false,
                1);
        SparkMaxConfig followConfig = new SparkMaxConfig(
                new SparkMaxStatusFrames(
                        20,
                        20,
                        20,
                        500,
                        500,
                        500,
                        500),
                0,
                false,
                IdleMode.kBrake,
                30,
                30,
                false, motor1);
        SparkMaxSetup.setup(motor1, leadConfig);
        SparkMaxSetup.setup(motor2, followConfig);
    }

    public void setPosition(double position) {
        motor1.getPIDController().setReference(position, ControlType.kPosition);
    }

    public void setPosition(ClimbPositions climbPositions) {
        setPosition(climbPositions.position);
    }

    public double getPosition() {
        return motor1.getEncoder().getPosition();
    }

    public double zero() {
        while (!limSwitch) {
            motor1.setVoltage(-0.1);
        }
        motor1.stopMotor();

        return motor1.getEncoder().getPosition() / gearing;
    }

    public void setOffset() {
        motor1.getEncoder().setPosition(0);
        setPosition(0);
    }
}
