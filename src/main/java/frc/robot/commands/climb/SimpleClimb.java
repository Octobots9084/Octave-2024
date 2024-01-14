package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ClimbPositions;

public class SimpleClimb extends SequentialCommandGroup{
    public SimpleClimb() {
        addCommands(
            new MoveClimb(ClimbPositions.DOWN).withTimeout(1),
            new MoveClimb(ClimbPositions.UP).withTimeout(2),
            new WaitCommand(3),
            new MoveClimb(ClimbPositions.DOWN));
    }
}
