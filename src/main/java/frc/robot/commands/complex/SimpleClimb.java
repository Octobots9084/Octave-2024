package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ClimbPositions;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class SimpleClimb extends SequentialCommandGroup{
    public SimpleClimb() {
        addCommands(
            new InstantCommand(() -> {
                    System.out.println("SimpleClimb requested");
                }),
            new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(1),
            new ClimbPosTolerance(ClimbPositions.DOWN),
            new InstantCommand(() -> {
                    Light.getInstance().setAnimation(Animations.DEFAULT);
                    System.out.println("SimpleClimb Complete");
                }));
    }
}
