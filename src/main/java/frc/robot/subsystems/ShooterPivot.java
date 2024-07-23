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

    public static double minLimit = 0.393;
    public static double maxLimit = 0.92;
    public CANSparkMax leftMotor, rightMotor;
    private double position;
    public boolean notSoFastEggman = false;
    public double desiredPos;

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
                new PIDConfig(10, 0.01, 0, 0.04));
        SparkMaxConfig left = new SparkMaxConfig(new SparkMaxStatusFrames(500,
                20,
                500,
                500,
                500,
                20,
                500), 1000, true,
                SparkMaxEncoderType.Absolute, IdleMode.kCoast, 30, 30, false, false, 1, false,
                new PIDConfig(10, 0.01, 0, 0.04));

        SparkMaxSetup.setup(leftMotor, left);
        SparkMaxSetup.setup(rightMotor, right);
        leftMotor.getPIDController().setIMaxAccum(20, 0);
        rightMotor.getPIDController().setIMaxAccum(20, 0);
        leftMotor.getPIDController().setIZone(2000);
        rightMotor.getPIDController().setIZone(2000);
    }

    public void setPosition(double target) {
        desiredPos = target;
        target = MathUtil.clamp(target, minLimit, maxLimit);
        // SmartDashboard.putNumber("targetPivot", target);
        // SmartDashboard.putNumber("realPivot", );
        // up is down on the encoders. If something is going up too high, lower the
        // number. - Xanthe
        leftMotor.getPIDController().setReference(target + .072, ControlType.kPosition);

        rightMotor.getPIDController().setReference(target + .06, ControlType.kPosition);
        position = target;
    }

    public void setPosition(ArmPositions armPositions) {

        setPosition(armPositions.pivot);
    }

    public double getPosition() {
        return leftMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() - .072;
    }

    public double getDesiredPosition() {
        return desiredPos;
    }

    public double getLastTargetPosition() {
        return position;
    }

    public void setIdleMode(IdleMode idleMode) {
        rightMotor.setIdleMode(idleMode);
        leftMotor.setIdleMode(idleMode);
    }

}
