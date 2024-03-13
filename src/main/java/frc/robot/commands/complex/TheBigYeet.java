package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.commands.intake.IntakeTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class TheBigYeet extends SequentialCommandGroup {
    public TheBigYeet() {
        addCommands(
                new InstantCommand(() -> {
                    ShooterTrack.getInstance().currentlyShooting = true;
                }),
                new ShooterTrackSpeedInstant(ShooterSpeeds.AMP),
                new IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT),
                new WaitCommand(0.2),
                new ShooterTrackSpeedInstant(ShooterSpeeds.STOP),
                new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                new ShooterPivotPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                new InstantCommand(() -> {
                    Light.getInstance().setAnimation(Animations.DEFAULT);
                }),
                new InstantCommand(() -> {
                    ShooterTrack.getInstance().currentlyShooting = false;
                }));
    }
}
