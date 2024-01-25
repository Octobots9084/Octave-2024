package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class Dunk extends SequentialCommandGroup{
    public Dunk() {
        addCommands(
            new ShooterTrackSpeedInstant(ShooterSpeeds.TRAP),
            new ShooterElevatorPosTolerance(ArmPositions.TRAP_SEGUEAY),
            new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY),
            new ShooterElevatorPosTolerance(ArmPositions.TRAP),
            new ShooterPivotPosTolerance(ArmPositions.TRAP),
            new ClimbPosTolerance(ClimbPositions.DOWN)
        );
    }
}
