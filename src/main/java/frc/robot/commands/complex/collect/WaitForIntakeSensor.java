package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeTrack;

public class WaitForIntakeSensor extends Command {
  @Override
  public boolean isFinished() {

    return !IntakeTrack.getInstance().getAnalogDigital();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake sensor triggered");
  }
}
