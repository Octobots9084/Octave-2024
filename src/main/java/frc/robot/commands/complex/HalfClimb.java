package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ClimbPositions;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class HalfClimb extends SequentialCommandGroup{
    public HalfClimb() {
        addCommands(
            new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(1),
            new ClimbPosTolerance(ClimbPositions.MID),
            new InstantCommand(() -> {
                Light.getInstance().setAnimation(Animations.CLIMB);
            }),
            new InstantCommand(() -> {
                System.out.println("Climb set to half.");
            }));
    }
}
