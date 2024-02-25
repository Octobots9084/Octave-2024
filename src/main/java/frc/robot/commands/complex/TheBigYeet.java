package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TheBigYeet extends SequentialCommandGroup {
    public TheBigYeet() {
        addCommands(
                new ShooterTrackSpeedInstant(ShooterSpeeds.AMP),
                new WaitCommand(0.5),
                new ShooterTrackSpeedInstant(ShooterSpeeds.STOP),
                new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                new ShooterPivotPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT),
                new InstantCommand(() -> {
                    ShooterFlywheel.getInstance().setFlywheelActive(false);
                }));
    }
}
