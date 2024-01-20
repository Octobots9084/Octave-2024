package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
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
        // Moving the joystick up moves the claw down so that the robot goes up and vice versa.
        // If we want to inverse the direction, change the sign below. 
        final var delta =
            0.1 *
            Math.signum(MathUtil.fitDeadband(ControlMap.CO_DRIVER_RIGHT.getY(), 0.01));

        var newPos = climb.getPosition() + delta;
        
        // Make sure the climb position is within the allowed range.
        newPos = edu.wpi.first.math.MathUtil.clamp(
            newPos,
            ClimbPositions.DOWN.position,
            ClimbPositions.UP.position);
        climb.setPosition(newPos);
    }
}
