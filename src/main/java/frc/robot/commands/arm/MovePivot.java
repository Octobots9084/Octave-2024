package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.Pivot;
import frc.robot.util.MathUtil;

public class MovePivot extends Command{
    ArmPositions armPositions;
    Pivot pivot;
    public MovePivot(ArmPositions armPositions) {
        this.armPositions = armPositions;
        pivot = Pivot.getInstance();
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