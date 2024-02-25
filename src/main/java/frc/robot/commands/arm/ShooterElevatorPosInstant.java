package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.ShooterElevator;

public class ShooterElevatorPosInstant extends InstantCommand {
    ArmPositions armPositions;
    ShooterElevator shooterElevator;

    public ShooterElevatorPosInstant(ArmPositions armPositions) {
        this.armPositions = armPositions;
        shooterElevator = ShooterElevator.getInstance();

    }

    @Override
    public void initialize() {
        shooterElevator.setPosition(armPositions);
        SmartDashboard.putNumber("elevator", armPositions.elevator);
    }

}