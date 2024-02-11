package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDConfig;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxEncoderType;
import frc.robot.util.SparkMax.SparkMaxSetup;
import frc.robot.util.SparkMax.SparkMaxStatusFrames;
import frc.robot.constants.IntakeSpeeds;

public class IntakeRoller extends SubsystemBase {
    private static IntakeRoller intake;

    public static IntakeRoller getInstance() {
        if (intake == null) {
            intake = new IntakeRoller();
        }
        return intake;
    }

    CANSparkMax motor1;

    public IntakeRoller() {
        motor1 = new CANSparkMax(10, MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig(new SparkMaxStatusFrames(500,
                20,
                500,
                20,
                500,
                500,
                500),
                1000,
                true, SparkMaxEncoderType.Relative, IdleMode.kBrake, 30, 30, false, false, 1, false,
                new PIDConfig(0, 0, 0, 0));
        SparkMaxSetup.setup(motor1, motorConfig);
    }

    public void setVelocity(double speed) {
        motor1.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    public void setVelocity(IntakeSpeeds speeds) {
        setVelocity(speeds.roller);
    }
}
