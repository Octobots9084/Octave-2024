package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
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

    public static double minLimit = 0.393;
    public static double maxLimit = 0.92;
    public CANSparkMax leftMotor, rightMotor;
    private double position;
    public boolean notSoFastEggman = false;

    public ShooterPivot() {
        // leadMotor and followMotor are outdated names, i'm changing it to left and
        // right
        rightMotor = new CANSparkMax(14, MotorType.kBrushless);
        leftMotor = new CANSparkMax(13, MotorType.kBrushless);
        SparkMaxConfig right = new SparkMaxConfig(new SparkMaxStatusFrames(500,
                20,
                500,
                500,
                500,
                20,
                500), 1000, true,
                SparkMaxEncoderType.Absolute, IdleMode.kCoast, 30, 30, true, false, 1, false,
                new PIDConfig(10, 0, 0, 0.04));
        SparkMaxConfig left = new SparkMaxConfig(new SparkMaxStatusFrames(500,
                20,
                500,
                500,
                500,
                20,
                500), 1000, true,
                SparkMaxEncoderType.Absolute, IdleMode.kCoast, 30, 30, false, false, 1, false,
                new PIDConfig(10, 0, 0, 0.04));

        SparkMaxSetup.setup(leftMotor, left);
        SparkMaxSetup.setup(rightMotor, right);
        leftMotor.getPIDController().setIMaxAccum(8, 0);
        rightMotor.getPIDController().setIMaxAccum(8, 0);
        leftMotor.getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(0.2 - (1/3));
    }

    public void setPosition(double target) {

        target = MathUtil.clamp(target, minLimit, maxLimit);
        // SmartDashboard.putNumber("targetPivot", target);
        leftMotor.getPIDController().setReference(target, ControlType.kPosition);

        rightMotor.getPIDController().setReference(target, ControlType.kPosition);
        position = target;
    }

    public void setPosition(ArmPositions armPositions) {

        setPosition(armPositions.pivot);
    }

    public double getPosition() {
        return leftMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

    public double getLastTargetPosition() {
        return position;
    }

    public void setIdleMode(IdleMode idleMode) {
        rightMotor.setIdleMode(idleMode);
        leftMotor.setIdleMode(idleMode);
    }

}
