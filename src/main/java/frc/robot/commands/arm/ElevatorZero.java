package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.ShooterElevator;

public class ElevatorZero extends Command {
    ArmPositions armPositions;
    ShooterElevator elevator;

    public ElevatorZero() {
        elevator = ShooterElevator.getInstance();
        super.addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.zero();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setOffset();
    }
}
