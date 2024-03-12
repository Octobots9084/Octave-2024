package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstantMeters;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class Dunk extends SequentialCommandGroup {
    public Dunk() {
        addCommands(
                new ShooterFlywheelSpeedInstantMeters(ShooterSpeeds.TRAP),
                new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY),
                new ShooterElevatorPosTolerance(ArmPositions.TRAP_SEGUEAY).withTimeout(5),
                new ClimbPosTolerance(ClimbPositions.DOWN).withTimeout(3),
                new ShooterPivotPosTolerance(ArmPositions.TRAP).withTimeout(3),
                new ShooterElevatorPosTolerance(ArmPositions.TRAP).withTimeout(3),
                new ShooterTrackSpeedInstant(ShooterSpeeds.TRAP));
    }
}
