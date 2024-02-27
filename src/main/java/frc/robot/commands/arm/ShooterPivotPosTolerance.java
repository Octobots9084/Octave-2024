package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.util.MathUtil;

public class ShooterPivotPosTolerance extends Command {
    ArmPositions armPositions;
    ShooterPivot pivot;

    public ShooterPivotPosTolerance(ArmPositions armPositions) {
        this.armPositions = armPositions;
        pivot = ShooterPivot.getInstance();
    }

    @Override
    public void initialize() {
        pivot.setPosition(armPositions);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("pivot pos", pivot.getPosition());
        return MathUtil.isWithinTolerance(pivot.getPosition(), armPositions.pivot,
                Constants.Arm.SHOOTER_PIVOT_TOLERANCE);
    }
}