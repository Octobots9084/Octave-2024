package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterFlywheelSpeedToleranceMeters;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class InitalSpeakerAuto extends SequentialCommandGroup {
    public InitalSpeakerAuto() {
        addCommands(
                new ShooterPivotPosTolerance(ArmPositions.SPEAKER_SHOT),
                new ShooterFlywheelSpeedToleranceMeters(ShooterSpeeds.SPEAKER),
                new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),

                new TheBigYeet()

        );
    }
}
