package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class CollectManualTwo extends SequentialCommandGroup {
    public CollectManualTwo() {

        BooleanSupplier intakeSensorTrue = () -> !IntakeTrack.getInstance().getSensor();
        BooleanSupplier shooterSensorTrue = () -> !ShooterTrack.getInstance().getSensor();
        BooleanSupplier shooterSensorNotTrue = () -> ShooterTrack.getInstance().getSensor();
        addCommands(
                        new InstantCommand(() -> {
                            Light.getInstance().setAnimation(Animations.INTAKE_STAGE_1);
                        }),
                        new IntakeRollerSpeedInstant(IntakeSpeeds.STOP),
                        new IntakeTrackSpeedInstant(IntakeSpeeds.STOP),
                        new ShooterPivotPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                        new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                        new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                        new ShooterTrackSpeedInstant(ShooterSpeeds.PREPARE));
    }

}
