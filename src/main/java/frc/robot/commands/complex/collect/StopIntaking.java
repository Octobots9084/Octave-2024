package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterPivot;

public class StopIntaking extends InstantCommand {
  @Override
  public void initialize() {
    IntakeRoller.getInstance().set(IntakeSpeeds.STOP);
    IntakeTrack.getInstance().set(IntakeSpeeds.STOP.track);
    System.out.println("Stopped Intaking");
  }
}
