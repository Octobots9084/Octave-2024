package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.JiggleNote;
import frc.robot.commands.complex.collect.DriverHasANote;
import frc.robot.commands.complex.collect.StartIntaking;
import frc.robot.commands.complex.collect.StartPassing;
import frc.robot.commands.complex.collect.StopIntaking;
import frc.robot.commands.complex.collect.StopPassing;
import frc.robot.commands.complex.collect.TriggerLightsCollectFinished;
import frc.robot.commands.complex.collect.TriggerLightsOnFirstSensors;
import frc.robot.commands.complex.collect.WaitForIntakeSensor;
import frc.robot.commands.complex.collect.WaitForPivot;
import frc.robot.commands.complex.collect.WaitForShooterSensor;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterTrack;

public class Collect extends SequentialCommandGroup {
  public Collect() {
    BooleanSupplier shooterHoldingNote = () -> ShooterTrack.getInstance().getSensor();
    addCommands(
        new ConditionalCommand(new SequentialCommandGroup(
            new StartIntaking(),
            new ParallelCommandGroup(new SequentialCommandGroup(new WaitForIntakeSensor(),
                new StopIntaking(),
                new WaitForPivot(),
                new StartPassing(),
                new WaitForShooterSensor(),
                new StopPassing(),
                new InstantCommand(() -> {
                  ShooterFlywheel.getInstance().setFlyWheelSpeedMeters(ShooterSpeeds.SPEAKER.flywheels);
                }),
                new TriggerLightsCollectFinished()), new TriggerLightsOnFirstSensors())),
            new DriverHasANote(),
            shooterHoldingNote)

    );
  }
}
