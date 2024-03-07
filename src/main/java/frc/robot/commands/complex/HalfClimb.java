package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ClimbPositions;

public class HalfClimb extends SequentialCommandGroup{
    public HalfClimb() {
        addCommands(
            new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(1),
            new ClimbPosTolerance(ClimbPositions.MID));
    }
}
