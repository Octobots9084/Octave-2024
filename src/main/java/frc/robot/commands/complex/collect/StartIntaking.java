package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ButtonConfig;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.vision.PieceVision;

public class StartIntaking extends InstantCommand {
  @Override
  public void initialize() {
    PieceVision.getInstance().setIsUsingPieceVision(true);
    IntakeRoller.getInstance().set(IntakeSpeeds.COLLECT);
    IntakeTrack.getInstance().setVoltage(IntakeSpeeds.COLLECT.track);
    ShooterPivot.getInstance().notSoFastEggman = true;
    ShooterPivot.getInstance().setPosition(ArmPositions.HANDOFF_AND_DEFAULT_SHOT);
    ShooterElevator.getInstance().setPosition(ArmPositions.HANDOFF_AND_DEFAULT_SHOT);
    System.out.println("Started Intaking");
  }
}
