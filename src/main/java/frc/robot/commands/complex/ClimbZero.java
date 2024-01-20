package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;

public class ClimbZero extends SequentialCommandGroup {
    public ClimbZero() {
        addCommands(
            new ClimbPosTolerance(ClimbPositions.DOWN).withTimeout(2)
        );
    }
}
