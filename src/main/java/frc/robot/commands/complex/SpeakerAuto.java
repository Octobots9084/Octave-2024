package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.arm.ShooterFlywheelSpeedTolerance;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class SpeakerAuto extends SequentialCommandGroup {
    public SpeakerAuto() {
        addCommands(
                new ShooterPivotPosTolerance(ArmPositions.SPEAKER_SHOT),
                new ShooterFlywheelSpeedTolerance(ShooterSpeeds.SPEAKER),
                new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),

                new TheBigYeet()

        );
    }
}
