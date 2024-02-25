package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.ShooterPivot;

public class ShooterPivotPosInstant extends InstantCommand {
    ArmPositions armPositions;
    ShooterPivot pivot;

    public ShooterPivotPosInstant(ArmPositions armPositions) {
        this.armPositions = armPositions;
        pivot = ShooterPivot.getInstance();
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("armpivo", armPositions.pivot);
        pivot.setPosition(armPositions);
    }
}