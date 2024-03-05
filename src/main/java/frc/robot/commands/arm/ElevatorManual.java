package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.ClimbPositions;
import frc.robot.robot.ControlMap;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.util.MathUtil;

public class ElevatorManual extends Command {

    ShooterElevator elev;

    public ElevatorManual() {
        elev = ShooterElevator.getInstance();
        addRequirements(elev);
    }

    @Override
    public void execute() {
        // Moving the joystick up moves the claw down so that the robot goes up and vice
        // versa.
        // If we want to inverse the direction, change the sign below.
        final var leftDelta = 0.05 *
                Math.signum(MathUtil.fitDeadband(ControlMap.CO_DRIVER_LEFT.getY(), Constants.Arm.SHOOTER_ELEVATOR_TOLERANCE));

        var newPos = elev.getPosition() + leftDelta;

        // Make sure the climb position is within the allowed range.
        newPos = edu.wpi.first.math.MathUtil.clamp(
                newPos,
                elev.min,
                elev.max);
        elev.setPosition(newPos);
    }
}
