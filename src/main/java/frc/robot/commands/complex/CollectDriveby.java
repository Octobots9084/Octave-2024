package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterTrack;

public class CollectDriveby extends SequentialCommandGroup{
    public CollectDriveby() {
        addCommands(
            new CollectAuto(),
            new DrivebyAuto(false)
        );
    }
}
