package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;

public class WaitForShooterSensor extends Command {
  @Override
  public boolean isFinished() {
    return !ShooterTrack.getInstance().getSensor();
  }

  @Override
  public void end(boolean interrupted) {
    ShooterPivot.getInstance().notSoFastEggman = false;
  }
}
