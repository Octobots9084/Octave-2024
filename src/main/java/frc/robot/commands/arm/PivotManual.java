package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.robot.ControlMap;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.util.MathUtil;

public class PivotManual extends Command{
    ShooterPivot piv;
    Joystick joystick;

    public PivotManual() {
        piv = ShooterPivot.getInstance();
        joystick = new Joystick(5);
        addRequirements(piv);
    }

    @Override
    public void execute() {
        if (joystick.getRawButton(13)) {
            // Moving the joystick up moves the claw down so that the robot goes up and vice
            // versa.
            // If we want to inverse the direction, change the sign below.
            final var leftDelta = 0.001 *
                    Math.signum(MathUtil.fitDeadband(ControlMap.CO_DRIVER_RIGHT.getX(), 0.1));

            var newPos = piv.getPosition() + leftDelta;

            // Make sure the climb position is within the allowed range.
            newPos = edu.wpi.first.math.MathUtil.clamp(
                    newPos,
                    ShooterPivot.minLimit,
                    ShooterPivot.maxLimit);
            piv.setPosition(newPos);
        }
        
    }
}
