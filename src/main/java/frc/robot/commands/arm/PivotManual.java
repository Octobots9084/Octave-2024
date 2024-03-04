package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.robot.ControlMap;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.util.MathUtil;

public class PivotManual extends Command{
    ShooterPivot piv;

    public PivotManual() {
        piv = ShooterPivot.getInstance();
        addRequirements(piv);
    }

    @Override
    public void execute() {
        // Moving the joystick up moves the claw down so that the robot goes up and vice
        // versa.
        // If we want to inverse the direction, change the sign below.
        final var leftDelta = 0.05 *
                Math.signum(MathUtil.fitDeadband(ControlMap.CO_DRIVER_LEFT.getY(), Constants.Arm.SHOOTER_PIVOT_TOLERANCE));

        var newPos = piv.getPosition() + leftDelta;

        // Make sure the climb position is within the allowed range.
        newPos = edu.wpi.first.math.MathUtil.clamp(
                newPos,
                0.435,
                0.69);
        piv.setPosition(newPos);
    }
}
