package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstantForce;
import frc.robot.commands.arm.ShooterTrackSpeedInstantForce;
import frc.robot.commands.intake.IntakeRollerSpeedInstant;
import frc.robot.commands.intake.IntakeTrackSpeedInstant;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class Panic extends ParallelCommandGroup {
    public Panic() {
        addCommands(
                new InstantCommand(() -> {
                    Light.getInstance().setAnimation(Animations.CLIMB);
                }),
                new ShooterFlywheelSpeedInstantForce(ShooterSpeeds.PANIC),
                new IntakeRollerSpeedInstant(IntakeSpeeds.PANIC),
                new IntakeTrackSpeedInstant(IntakeSpeeds.PANIC),
                new ShooterTrackSpeedInstantForce(ShooterSpeeds.PANIC));
    }
}
