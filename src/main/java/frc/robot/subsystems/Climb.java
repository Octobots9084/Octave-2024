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

    CANSparkMax leftMotor, rightMotor;
    public boolean limSwitch;
    private double gearing = 125;
    private double extensionDistance = 0.52;

    public Climb() {
        leftMotor = new CANSparkMax(18, MotorType.kBrushless);
        rightMotor = new CANSparkMax(19, MotorType.kBrushless);
        SparkMaxConfig leftConfig = new SparkMaxConfig(
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
        SparkMaxConfig rightConfig = new SparkMaxConfig(
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
                true,
                false,
                1);
        SparkMaxSetup.setup(leftMotor, leftConfig);
        SparkMaxSetup.setup(rightMotor, rightConfig);
        leftMotor.getEncoder().setPositionConversionFactor(1.0/gearing);
        rightMotor.getEncoder().setPositionConversionFactor(1.0/gearing);
    }

    public void setPosition(double position) {
        leftMotor.getPIDController().setReference(position, ControlType.kPosition);
        rightMotor.getPIDController().setReference(position, ControlType.kPosition);
    }

    public void setPosition(ClimbPositions climbPositions) {
        setPosition(climbPositions.position);
    }

    public double getPosition() {
        return (leftMotor.getEncoder().getPosition()+rightMotor.getEncoder().getPosition())/2.0;
    }

    public void zero() {
        // while (!limSwitch) {
        //     leftMotor.setVoltage(-0.1);
        // }
        // leftMotor.stopMotor();

        // return leftMotor.getEncoder().getPosition() / gearing;
    }

    public void setOffset() {
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
        setPosition(0);
    }
}
