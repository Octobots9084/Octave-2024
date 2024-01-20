package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class TheBigYeet extends SequentialCommandGroup {
    public TheBigYeet() {
        addCommands(
            new ShooterTrackSpeedInstant(ShooterSpeeds.AMP),
            new WaitCommand(3),
            new ShooterTrackSpeedInstant(ShooterSpeeds.STOP),
            new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
            new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT)
        );
    }
}
