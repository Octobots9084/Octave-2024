package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.ArmPositions;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.util.MathUtil;

public class WaitForPivot extends Command {
  @Override
  public boolean isFinished() {
    System.out.println("Waiting for pivot, " + (ShooterPivot.getInstance().getPosition() - 0.072) + ", boolean at: " + MathUtil.isWithinTolerance(
        ShooterPivot.getInstance().getPosition() - 0.072, ArmPositions.HANDOFF_AND_DEFAULT_SHOT.pivot,
        Constants.Arm.SHOOTER_PIVOT_TOLERANCE));
    return MathUtil.isWithinTolerance(
        ShooterPivot.getInstance().getPosition() - 0.072, ArmPositions.HANDOFF_AND_DEFAULT_SHOT.pivot,
        Constants.Arm.SHOOTER_PIVOT_TOLERANCE);

  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Pivot at handoff");
  }
}
