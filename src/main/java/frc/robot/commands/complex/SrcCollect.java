package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterTrack;

public class SrcCollect extends SequentialCommandGroup {
    public SrcCollect() {
        BooleanSupplier shooterSensorFalse = () -> ShooterTrack.getInstance().getSensor();
        BooleanSupplier shooterSensorTrue = () -> !ShooterTrack.getInstance().getSensor();
        if (shooterSensorTrue.getAsBoolean()) {
            return;
        }
        addCommands(
                new ShooterFlywheelSpeedInstant(ShooterSpeeds.SRC_COLLECT),
                new ShooterPivotPosInstant(ArmPositions.SOURCE_COLLECT),
                new WaitUntilCommand(shooterSensorTrue),
                new WaitUntilCommand(shooterSensorFalse),
                new WaitUntilCommand(shooterSensorTrue),
                new ShooterFlywheelSpeedInstant(ShooterSpeeds.STOP),
                new ShooterPivotPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT)
                );
    }

}
