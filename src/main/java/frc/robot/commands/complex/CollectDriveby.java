package frc.robot.commands.complex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CollectDriveby extends SequentialCommandGroup {
    public CollectDriveby() {
        // SmartDashboard.putNumber("Auto", SmartDashboard.getNumber("Auto", 0) + 1);
        System.out.println("collectdrivebyconst");
        addCommands(
                new InstantCommand(() -> {
                    System.out.println("collectdrvebyrun");
                }),
                new Collect(),
                new InstantCommand(() -> {
                    System.out.println("collectdrvebyrun2");
                }),
                new Driveby().withTimeout(1),
                // trigger time
                new WaitCommand(0.05),
                new InstantCommand(() -> {
                    System.out.println("collectdrvebyrun3");
                }));
    }
}
