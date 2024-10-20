package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.vision.PieceVision;

public class WaitForIntakeSensor extends Command {
  @Override
  public boolean isFinished() {

    return !IntakeTrack.getInstance().getAnalogDigital() || !IntakeTrack.getInstance().getSensor();
  }

  @Override
  public void end(boolean interrupted) {
    PieceVision.getInstance().setIsUsingPieceVision(false);
    System.out.println("Intake sensor triggered");
  }
}
