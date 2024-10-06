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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PrepClimb extends SequentialCommandGroup {
    public PrepClimb() {
        addCommands(
            new InstantCommand(() -> {
                System.out.println("Climb prep requested");
                SwerveSubsystem.getInstance().setTargetSpeaker(false);
            }),
            new InstantCommand(()->{
                if (ShooterPivot.getInstance().notSoFastEggman) {
                    System.out.println("Wait for collect to finish!");
                    return;
                }
                ShooterElevator.getInstance().setPosition(ArmPositions.PREP_TRAP);
            }),
                new ClimbPosTolerance(ClimbPositions.UP).withTimeout(4),
                new InstantCommand(() -> {
                    Light.getInstance().setAnimation(Animations.CLIMB);
                    if (!ShooterPivot.getInstance().notSoFastEggman) {
                        System.out.println("Wait for collect to finish! pt2");
                    } 
                    else {
                        ShooterPivot.getInstance().setPosition(ArmPositions.PREP_TRAP);
                    }
                    ShooterFlywheel.getInstance().setFlywheelSpeed(0);
                    System.out.println("Climb prepped");
                }));
    }
}
