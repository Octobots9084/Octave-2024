package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.constants.ArmPositions;
import frc.robot.util.PIDConfig;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxEncoderType;
import frc.robot.util.SparkMax.SparkMaxSetup;
import frc.robot.util.SparkMax.SparkMaxStatusFrames;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterElevator extends SubsystemBase {
    private static ShooterElevator shooterElevator;

    public static ShooterElevator getInstance() {
        if (shooterElevator == null) {
            shooterElevator = new ShooterElevator();
        }
        return shooterElevator;
    }

    CANSparkMax motor1, motor2;
    public boolean limSwitch;
    public final double gearing = 1;
    public double max = 44.8;
    public double min = 1;
    public double desiredpos;

    private SparkMaxConfig elevateConfig = new SparkMaxConfig(
            new SparkMaxStatusFrames(300, 300, 10, 300, 300, 300, 300), 1, true, SparkMaxEncoderType.Relative,
            IdleMode.kBrake,
            30, 30, false, false, 1, false, new PIDConfig(2, 0, 0));
    private SparkMaxConfig followConfig = new SparkMaxConfig(new SparkMaxStatusFrames(300, 300, 10, 300, 300, 300, 300),
            1, true, IdleMode.kBrake, 30, 30, false, motor1);

    public ShooterElevator() {
        motor1 = new CANSparkMax(12, MotorType.kBrushless);
        // motor2 = new CANSparkMax(0, MotorType.kBrushless);
        SparkMaxSetup.setup(motor1, elevateConfig);
        motor1.enableVoltageCompensation(5);
        // SparkMaxSetup.setup(motor2, followConfig);
        setOffset();
    }

    public double zero() {
        while (!limSwitch) {
            motor1.setVoltage(-0.1);
        }
        motor1.stopMotor();

        return motor1.getEncoder().getPosition() / gearing;
    }

    public void setPosition(double position) {
        desiredpos = position;
        motor1.getPIDController().setReference(position, ControlType.kPosition, 0, 1);
    }

    public void setPosition(ArmPositions armPositions) {
        setPosition(armPositions.elevator);
    }

    public double getDesiredPosition() {
        return desiredpos;
    }

    public double getPosition() {
        return motor1.getEncoder().getPosition();
    }

    public void goToMax() {
        motor1.getPIDController().setReference(gearing * max, ControlType.kPosition);
    }

    public void goToMin() {
        motor1.getPIDController().setReference(gearing * min, ControlType.kPosition);
    }

    public void setOffset() {
        motor1.getEncoder().setPosition(0);
        setPosition(0);
    }

}