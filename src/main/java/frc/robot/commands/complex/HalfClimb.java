package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ClimbPositions;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class HalfClimb extends SequentialCommandGroup {

    public HalfClimb() {
        BooleanSupplier eggBooleanSupplier = () -> {
            return ShooterPivot.getInstance().notSoFastEggman;
        };
        addCommands(
                new ConditionalCommand(
                        new InstantCommand(() -> {System.out.println("Collect isn't done Linus.");}),
                        new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(1),
                        eggBooleanSupplier),
                new ClimbPosTolerance(ClimbPositions.MID),
                new InstantCommand(() -> {
                    Light.getInstance().setAnimation(Animations.CLIMB);
                }),
                new InstantCommand(() -> {
                    System.out.println("Climb set to half.");
                }));
    }
}
