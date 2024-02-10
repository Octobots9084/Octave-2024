package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.commands.intake.IntakeRollerSpeedInstant;
import frc.robot.commands.intake.IntakeTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterTrack;

public class Collect extends SequentialCommandGroup {
    public Collect() {
        BooleanSupplier intakeSensorTrue = () -> IntakeTrack.getInstance().getSensor();
        BooleanSupplier shooterSensorTrue = () -> ShooterTrack.getInstance().getSensor();
        addCommands(
                new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                new IntakeRollerSpeedInstant(IntakeSpeeds.COLLECT),
                new WaitUntilCommand(intakeSensorTrue),
                new IntakeTrackSpeedInstant(IntakeSpeeds.REJECT),
                new IntakeRollerSpeedInstant(IntakeSpeeds.REJECT),
                new ParallelCommandGroup(new ShooterPivotPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                        new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT)),
                new ShooterTrackSpeedInstant(ShooterSpeeds.SPEAKER),
                new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                new WaitUntilCommand(shooterSensorTrue),
                new ShooterTrackSpeedInstant(ShooterSpeeds.STOP),
                new IntakeTrackSpeedInstant(IntakeSpeeds.STOP));
    }
}
