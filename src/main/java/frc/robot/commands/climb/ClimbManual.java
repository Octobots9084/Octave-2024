package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.ClimbPositions;
import frc.robot.robot.ControlMap;
import frc.robot.subsystems.Climb;
import frc.robot.util.MathUtil;

public class ClimbManual extends Command {

    Climb climb;

    public ClimbManual() {
        climb = Climb.getInstance();
        addRequirements(climb);
    }

    @Override
    public void execute() {
        // Moving the joystick up moves the claw down so that the robot goes up and vice
        // versa.
        // If we want to inverse the direction, change the sign below.
        final var leftDelta = 0.05 *
                Math.signum(MathUtil.fitDeadband(ControlMap.CO_DRIVER_LEFT.getY(), Constants.Climb.MANUAL_DEADBAND));
        final var rightDelta = 0.05 *
                Math.signum(MathUtil.fitDeadband(ControlMap.CO_DRIVER_RIGHT.getY(), Constants.Climb.MANUAL_DEADBAND));

        var newLeftPos = climb.getLeftPosition() + leftDelta;
        var newRightPos = climb.getRightPosition() + rightDelta;

        // Make sure the climb position is within the allowed range.
        newLeftPos = edu.wpi.first.math.MathUtil.clamp(
                newLeftPos,
                ClimbPositions.DOWN.leftPosition,
                ClimbPositions.UP.leftPosition);

        newRightPos = edu.wpi.first.math.MathUtil.clamp(
                newRightPos,
                ClimbPositions.DOWN.rightPosition,
                ClimbPositions.UP.rightPosition);
        climb.setPosition(newLeftPos, newRightPos);
    }
}
