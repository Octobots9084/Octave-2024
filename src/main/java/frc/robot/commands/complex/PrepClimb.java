package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ClimbPositions;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class PrepClimb extends SequentialCommandGroup {
    public PrepClimb() {
        addCommands(
                new ClimbPosTolerance(ClimbPositions.UP).withTimeout(10),
                new InstantCommand(() -> {
                    Light.getInstance().setAnimation(Animations.CLIMB);
                }),new SequentialCommandGroup(new InstantCommand(()->{
                ShooterElevator.getInstance().setPosition(ArmPositions.PREP_TRAP);
            }),
            new InstantCommand(()->{
                ShooterPivot.getInstance().setPosition(ArmPositions.PREP_TRAP);
            }),
            new InstantCommand(()->{
                ShooterFlywheel.getInstance().setFlywheelSpeed(0);
            })));
    }
}
