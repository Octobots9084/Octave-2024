package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmPositions;
import frc.robot.util.PIDConfig;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxEncoderType;
import frc.robot.util.SparkMax.SparkMaxSetup;
import frc.robot.util.SparkMax.SparkMaxStatusFrames;

public class ShooterPivot extends SubsystemBase {
    public static ShooterPivot pivot;

    public static ShooterPivot getInstance() {
        if (pivot == null) {
            pivot = new ShooterPivot();
        }
        return pivot;
    }

    private static double minLimit = 0.393;
    private static double maxLimit = 0.92;
    public CANSparkMax leadMotor, followMotor;
    private double position;

    public ShooterPivot() {
        leadMotor = new CANSparkMax(14, MotorType.kBrushless);
        followMotor = new CANSparkMax(13, MotorType.kBrushless);
        SparkMaxConfig follow = new SparkMaxConfig(new SparkMaxStatusFrames(500,
                20,
                500,
                500,
                500,
                20,
                500), 1000, true,
                SparkMaxEncoderType.Relative, IdleMode.kCoast, 30, 30, false, false, 1, false, new PIDConfig(3, 0, 0,0.05));
        SparkMaxConfig lead = new SparkMaxConfig(new SparkMaxStatusFrames(500,
                20,
                500,
                500,
                500,
                20,
                500), 1000, true,
                SparkMaxEncoderType.Relative, IdleMode.kCoast, 30, 30, true, false, 1, false, new PIDConfig(3, 0, 0,0.05));
        SparkMaxSetup.setup(leadMotor, lead);
        SparkMaxSetup.setup(followMotor, follow);
        leadMotor.getEncoder().setPositionConversionFactor(1.0/125);
        followMotor.getEncoder().setPositionConversionFactor(1.0/125);
        leadMotor.getEncoder().setPosition(leadMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        followMotor.getEncoder().setPosition(leadMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
        
        
    }

    public void setPosition(double target) {
        SmartDashboard.putNumber("targetPivot", target);
        target = MathUtil.clamp(target, minLimit, maxLimit);
        leadMotor.getPIDController().setReference(target, ControlType.kPosition);
        position = target;
    }

    public void setPosition(ArmPositions armPositions) {

        setPosition(armPositions.pivot);
    }

    public double getPosition() {
        return leadMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

    public double getLastTargetPosition() {
        return position;
    }

    public void setIdleMode(IdleMode idleMode) {
        leadMotor.setIdleMode(idleMode);
    }

}
