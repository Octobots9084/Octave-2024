package frc.robot.commands.complex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class CollectDriveby extends SequentialCommandGroup{
    public CollectDriveby() {
        SmartDashboard.putNumber("Auto", SmartDashboard.getNumber("Auto", 0) + 1);
        System.out.println("collectdrivebyconst");
        addCommands(
            new InstantCommand(() -> {
                    System.out.println("collectdrvebyrun");
            }),
            new CollectAuto(),
            new InstantCommand(() -> {
                    System.out.println("collectdrvebyrun2");
            }),
            new DrivebyAuto(false).withTimeout(1),
            new InstantCommand(() -> {
                    System.out.println("collectdrvebyrun3");
            })
        );
    }
}
