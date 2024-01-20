package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.util.MathUtil;

public class ShooterPivotPosTolerance extends Command{
    ArmPositions armPositions;
    ShooterPivot pivot;
    public ShooterPivotPosTolerance(ArmPositions armPositions) {
        this.armPositions = armPositions;
        pivot = ShooterPivot.getInstance();
        super.addRequirements(pivot);
    }

    @Override 
    public void initialize() {
        pivot.setPosition(armPositions);
    }

    @Override 
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(pivot.getPosition(), armPositions.pivot, 0.1);
    }
}