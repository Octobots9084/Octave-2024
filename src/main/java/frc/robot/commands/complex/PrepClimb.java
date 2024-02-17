package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;

public class PrepClimb extends SequentialCommandGroup {
    public PrepClimb() {
        addCommands(
                new ClimbPosTolerance(ClimbPositions.UP).withTimeout(10));
    }
}
