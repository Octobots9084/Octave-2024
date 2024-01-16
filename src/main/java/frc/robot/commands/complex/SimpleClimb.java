package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;

public class SimpleClimb extends SequentialCommandGroup{
    public SimpleClimb() {
        addCommands(
            new ClimbPosTolerance(ClimbPositions.DOWN).withTimeout(1),
            new ClimbPosTolerance(ClimbPositions.UP).withTimeout(2),
            new WaitCommand(3),
            new ClimbPosTolerance(ClimbPositions.DOWN));
    }
}
