package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.constants.ArmPositions;

public class Undunk extends SequentialCommandGroup{
    public Undunk() {
        addCommands(
            new InstantCommand(() -> {
                    Light.getInstance().setAnimation(Animations.CLIMB);
                }),
                new InstantCommand(() -> {
                    System.out.println("Undunck requested");
                }),
            new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(3),
            new ParallelCommandGroup(
            new ShooterElevatorPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(3),
            new ClimbPosTolerance(ClimbPositions.UP).withTimeout(3),
            new InstantCommand(() -> {
                System.out.println("Undunk complete hopefully");
            })
            )
        );
    }
}
