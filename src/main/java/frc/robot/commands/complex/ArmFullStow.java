package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.constants.ArmPositions;

public class ArmFullStow extends SequentialCommandGroup {
    public ArmFullStow() {
        addCommands(
            new ShooterPivotPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
            new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT)
        );
    }
}
