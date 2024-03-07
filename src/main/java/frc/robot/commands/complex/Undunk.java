package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.constants.ArmPositions;

public class Undunk extends SequentialCommandGroup{
    public Undunk() {
        addCommands(
            new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(3),
            new ParallelCommandGroup(
            new ShooterElevatorPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(3),
            new ClimbPosTolerance(ClimbPositions.UP).withTimeout(3)
            )
        );
    }
}
