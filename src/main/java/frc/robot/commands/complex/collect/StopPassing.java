package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterTrack;

public class StopPassing extends InstantCommand {
  @Override
  public void initialize() {
    IntakeRoller.getInstance().set(IntakeSpeeds.STOP);
    IntakeTrack.getInstance().setVoltage(IntakeSpeeds.STOP.track);
    ShooterTrack.getInstance().setVoltage(ShooterSpeeds.STOP.track);
    System.out.println("Stopped Passing");
  }
}
