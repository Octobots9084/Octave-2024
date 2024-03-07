package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.constants.ClimbPositions;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstantMeters;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterFlywheel;

public class Layup extends SequentialCommandGroup {
    public Layup() {
        addCommands(
                new ShooterPivotPosTolerance(ArmPositions.TRAP_SEGUEAY),
                new ShooterElevatorPosTolerance(ArmPositions.LAYUP).withTimeout(5),
                new InstantCommand(() -> ShooterFlywheel.getInstance().setFlywheelActive(false)),
                new ClimbPosTolerance(ClimbPositions.DOWN).withTimeout(3),
                new ShooterPivotPosTolerance(ArmPositions.LAYUP),
                new WaitCommand(0.25),
                new ShooterTrackSpeedInstant(ShooterSpeeds.LAYUP),
                new WaitCommand(3),
                new ShooterPivotPosInstant(ArmPositions.TRAP_SEGUEAY)
                );
    }
}
