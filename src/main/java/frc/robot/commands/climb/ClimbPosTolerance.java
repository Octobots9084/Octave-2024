package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.ClimbPositions;
import frc.robot.subsystems.Climb;
import frc.robot.util.MathUtil;

public class ClimbPosTolerance extends Command {
    ClimbPositions climbPositions;
    Climb climb;

    public ClimbPosTolerance(ClimbPositions climbPositions) {
        this.climbPositions = climbPositions;
        climb = Climb.getInstance();
    }

    @Override
    public void initialize() {
        climb.setPosition(climbPositions);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(climb.getLeftPosition(), climbPositions.leftPosition,
                Constants.Climb.CLIMB_TOLERANCE)
                && MathUtil.isWithinTolerance(climb.getRightPosition(), climbPositions.rightPosition,
                        Constants.Climb.CLIMB_TOLERANCE);
    }
}
