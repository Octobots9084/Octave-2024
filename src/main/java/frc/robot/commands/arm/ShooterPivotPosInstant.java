package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.ShooterPivot;

public class ShooterPivotPosInstant extends InstantCommand{
    ArmPositions armPositions;
    ShooterPivot pivot;
    
    public ShooterPivotPosInstant(ArmPositions armPositions) {
        this.armPositions = armPositions;
        pivot = ShooterPivot.getInstance();
        super.addRequirements(pivot);
    }

    @Override 
    public void initialize() {
        pivot.setPosition(armPositions);
    }
}