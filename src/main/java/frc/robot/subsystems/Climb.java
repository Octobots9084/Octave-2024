package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDConfig;
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
    public static final double max = 38;

    public Climb() {
        leftMotor = new CANSparkMax(40, MotorType.kBrushless);
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
                IdleMode.kCoast,
                10,
                10,
                false,
                false,
                1, false, new PIDConfig(1, 0, 0));
        SparkMaxSetup.setup(leftMotor, leftConfig);
        setOffset();
    }

    public void setPosition(double leftPosition, double rightPosition) {
        leftMotor.getPIDController().setReference(leftPosition, ControlType.kPosition);
    }


    public double getLeftPosition() {
        return leftMotor.getEncoder().getPosition();
    }


    public void zero() {
        // while (!limSwitch) {
        // leftMotor.setVoltage(-0.1);
        // }
        // leftMotor.stopMotor();

        // return leftMotor.getEncoder().getPosition() / gearing;
    }

    public void setOffset() {
        leftMotor.getEncoder().setPosition(0);
        setPosition(0, 0);
    }
}