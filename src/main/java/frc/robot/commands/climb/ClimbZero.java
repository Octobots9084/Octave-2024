package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbZero extends Command {
    Climb climb;

    public ClimbZero() {
        climb = Climb.getInstance();
        super.addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.zero();
    }

    @Override
    public void end(boolean interrupted) {
        climb.setOffset();
    }
}
