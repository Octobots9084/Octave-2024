package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstantMeters;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class OldFerryShot extends SequentialCommandGroup{
    public OldFerryShot() {
        addCommands(
            new ShooterFlywheelSpeedInstantMeters(ShooterSpeeds.FERRY_SHOT),
            new ShooterElevatorPosInstant(ArmPositions.FERRY_SHOT),
            new ShooterPivotPosTolerance(ArmPositions.FERRY_SHOT).withTimeout(0.25),
            new TheBigYeet()
        );}}
