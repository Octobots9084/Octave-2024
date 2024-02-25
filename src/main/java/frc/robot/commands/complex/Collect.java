package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.JiggleNote;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.commands.intake.IntakeRollerSpeedInstant;
import frc.robot.commands.intake.IntakeTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;

public class Collect extends SequentialCommandGroup {
    public Collect() {
        BooleanSupplier intakeSensorTrue = () -> !IntakeTrack.getInstance().getSensor();
        BooleanSupplier shooterSensorTrue = () -> !ShooterTrack.getInstance().getSensor();
        if (shooterSensorTrue.getAsBoolean()) {
            return;
        }
        addCommands(
                new InstantCommand(() -> {
                    SmartDashboard.putBoolean("CollectRunning", true);
                }),
                new InstantCommand(() -> {
                    ShooterPivot.getInstance().notSoFastEggman = true;
                }),
                new ShooterTrackSpeedInstant(ShooterSpeeds.IDLE),
                new ParallelCommandGroup(new ShooterPivotPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                        new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT)),
                new InstantCommand(() -> {
                    SmartDashboard.putBoolean("reached Checkpoint", false);
                }),
                new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                new IntakeRollerSpeedInstant(IntakeSpeeds.COLLECT),
                new WaitUntilCommand(intakeSensorTrue),
                new InstantCommand(() -> {
                    SmartDashboard.putBoolean("reached Checkpoint", true);
                }),

                new IntakeRollerSpeedInstant(IntakeSpeeds.REJECT),
                new ParallelCommandGroup(new ShooterPivotPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                        new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT)),

                new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                new ShooterTrackSpeedInstant(ShooterSpeeds.PREPARE),

                new WaitUntilCommand(shooterSensorTrue),
                new IntakeTrackSpeedInstant(IntakeSpeeds.REJECT),
                new InstantCommand(() -> {
                    ShooterPivot.getInstance().notSoFastEggman = false;
                }),
                new ShooterTrackSpeedInstant(ShooterSpeeds.STOP),
                new IntakeTrackSpeedInstant(IntakeSpeeds.STOP),
                new JiggleNote(1));

    }

}
