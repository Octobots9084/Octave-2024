package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class Dunk extends SequentialCommandGroup {
    public Dunk() {
        addCommands(
                new ShooterFlywheelSpeedInstant(ShooterSpeeds.TRAP),
                new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY),
                new ShooterElevatorPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(5),
                new ClimbPosTolerance(ClimbPositions.DOWN),
                new ShooterPivotPosTolerance(ArmPositions.TRAP),
                new ShooterElevatorPosTolerance(ArmPositions.TRAP),
                new ShooterTrackSpeedInstant(ShooterSpeeds.TRAP));
    }
}
