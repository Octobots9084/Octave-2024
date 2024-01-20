package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class trap extends SequentialCommandGroup{
    public trap() {
        addCommands(
            new ClimbPosTolerance(ClimbPositions.DOWN).withTimeout(1),
            new ClimbPosTolerance(ClimbPositions.UP).withTimeout(2),
            new WaitCommand(3),
            new ShooterElevatorPosInstant(ArmPositions.AMP), 
            new ShooterFlywheelSpeedInstant(ShooterSpeeds.AMP),
            new ShooterPivotPosInstant(ArmPositions.AMP),
            new ClimbPosTolerance(ClimbPositions.DOWN)
        );
    }
}
// This is the most scufed thing ever there is 99% chances somethings busted I literally just frankensteined
// prepAmp and the version of SimpleClimb that I think Annie said was broken