package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterFlywheelSpeedToleranceMeters;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class SpeakerAutoSlow extends SequentialCommandGroup {
    public SpeakerAutoSlow() {
        addCommands(
                new ShooterPivotPosInstant(ArmPositions.SPEAKER_SHOT).withTimeout(1),
                new ShooterFlywheelSpeedToleranceMeters(ShooterSpeeds.LAYUP).withTimeout(1),
                new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT).withTimeout(1),

                new TheBigYeet()

        );
    }
}
