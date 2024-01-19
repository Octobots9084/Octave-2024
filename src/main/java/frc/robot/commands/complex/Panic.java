package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.commands.intake.IntakeRollerSpeedInstant;
import frc.robot.commands.intake.IntakeTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ClimbPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;

public class Panic extends ParallelCommandGroup{
    public Panic() {
        addCommands(
            new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
            new ShooterFlywheelSpeedInstant(ShooterSpeeds.PANIC),
            new ShooterPivotPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
            new ShooterTrackSpeedInstant(ShooterSpeeds.PANIC),
            new ClimbPosTolerance(ClimbPositions.DOWN),
            new IntakeRollerSpeedInstant(IntakeSpeeds.PANIC),
            new IntakeTrackSpeedInstant(IntakeSpeeds.PANIC)
        );
    }
}
