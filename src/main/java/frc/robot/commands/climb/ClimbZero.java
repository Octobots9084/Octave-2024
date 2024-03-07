package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbPositions;
import frc.robot.subsystems.Climb;

public class ClimbZero extends Command {
    Climb climb;

    public ClimbZero() {
        climb = Climb.getInstance();
    }

    @Override
    public void initialize() {
        climb.setLowCurrentLimits();
        climb.zero();
    }

    @Override
    public void end(boolean interrupted) {
        climb.setHighCurrentLimits();
       climb.setPosition(ClimbPositions.DOWN);
    }
}
