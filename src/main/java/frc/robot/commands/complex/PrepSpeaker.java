package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstantMeters;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class PrepSpeaker extends SequentialCommandGroup {
    public PrepSpeaker() {
        addCommands(
                new ShooterFlywheelSpeedInstantMeters(ShooterSpeeds.SPEAKER),
                new ShooterElevatorPosTolerance(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                new ShooterPivotPosInstant(ArmPositions.SPEAKER_SHOT)

        );
    }
}
