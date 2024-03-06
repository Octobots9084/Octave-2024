package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class FerryShot extends SequentialCommandGroup{
    public FerryShot() {
        addCommands(
            new ShooterFlywheelSpeedInstant(ShooterSpeeds.FERRY_SHOT),
            new ShooterElevatorPosInstant(ArmPositions.FERRY_SHOT),
            new ShooterPivotPosTolerance(ArmPositions.FERRY_SHOT).withTimeout(1),
            new ShooterTrackSpeedInstant(ShooterSpeeds.FERRY_SHOT)
        );
    }    
}
