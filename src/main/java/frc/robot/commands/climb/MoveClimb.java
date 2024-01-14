package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbPositions;
import frc.robot.subsystems.Climb;
import frc.robot.util.MathUtil;

public class MoveClimb extends Command{
    ClimbPositions climbPositions;
    Climb climb;
    public MoveClimb(ClimbPositions climbPositions) {
        this.climbPositions = climbPositions;
        climb = Climb.getInstance();
        super.addRequirements(climb);
    }

    @Override 
    public void initialize() {
        climb.setPosition(climbPositions);
    }

    @Override 
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(climb.getPosition(), climbPositions.position, 0.1);
    }
}
