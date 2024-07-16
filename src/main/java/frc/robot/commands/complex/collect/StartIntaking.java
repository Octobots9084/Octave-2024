package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterPivot;

public class StartIntaking extends InstantCommand {
  @Override
  public void initialize() {

    IntakeRoller.getInstance().set(IntakeSpeeds.COLLECT);
    IntakeTrack.getInstance().set(IntakeSpeeds.COLLECT);
    ShooterPivot.getInstance().notSoFastEggman = true;
    ShooterPivot.getInstance().setPosition(ArmPositions.HANDOFF_AND_DEFAULT_SHOT);
    System.out.println("Started Intaking");
  }
}
