package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterTrack;

public class CollectDrivebySafely extends SequentialCommandGroup {
    public CollectDrivebySafely() {
        BooleanSupplier shooterSensorTrue = () -> !ShooterTrack.getInstance().getSensor();
        // SmartDashboard.putNumber("Auto", SmartDashboard.getNumber("Auto", 0) + 1);
        System.out.println("collectdrivebyconst");
        addCommands(
                new InstantCommand(() -> {
                    System.out.println("collectdrvebyrun");
                }),
                new Collect().withTimeout(3),
                new InstantCommand(() -> {
                    System.out.println("collectdrvebyrun2");
                }),
                new ConditionalCommand(new DrivebyAuto(false).withTimeout(1.5), new InstantCommand(),
                        shooterSensorTrue),
                // trigger time
                new WaitCommand(0.2),
                new InstantCommand(() -> {
                    System.out.println("collectdrvebyrun3");
                }));
    }
}
