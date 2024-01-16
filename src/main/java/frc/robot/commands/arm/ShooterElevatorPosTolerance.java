package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.util.MathUtil;

public class ShooterElevatorPosTolerance extends Command{
    ArmPositions armPositions;
    ShooterElevator shooterElevator;
    public ShooterElevatorPosTolerance(ArmPositions armPositions) {
        this.armPositions = armPositions;
        shooterElevator = ShooterElevator.getInstance();
        super.addRequirements(shooterElevator);
    }

    @Override 
    public void initialize() {
        shooterElevator.setPosition(armPositions);
    }

    @Override 
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(shooterElevator.getPosition(), armPositions.elevator, 0.1);
    }
}