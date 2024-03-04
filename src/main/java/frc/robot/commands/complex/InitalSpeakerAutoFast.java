package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstantMeters;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.commands.arm.ShooterTrackPercentInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class InitalSpeakerAutoFast extends SequentialCommandGroup {
    public InitalSpeakerAutoFast() {

        addCommands(new ShooterPivotPosInstant(ArmPositions.SPEAKER_SHOT), new WaitCommand(0),
                new ShooterFlywheelSpeedInstantMeters(ShooterSpeeds.AUTO_SPEAKER), new WaitCommand(0),
                new ShooterTrackPercentInstant(1));
    }
}
