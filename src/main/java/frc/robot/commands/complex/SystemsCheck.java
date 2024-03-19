package frc.robot.commands.complex;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SystemsCheck extends SequentialCommandGroup{
    public SystemsCheck() {
        addCommands(
            new InstantCommand(() -> {SwerveSubsystem.getInstance().driveFieldOriented(new ChassisSpeeds(5, 0, 0));}),
            new WaitCommand(3),
            new InstantCommand(() -> {SwerveSubsystem.getInstance().driveFieldOriented(new ChassisSpeeds(0, 5, 0));}),
            new WaitCommand(3),
            new InstantCommand(() -> {SwerveSubsystem.getInstance().driveFieldOriented(new ChassisSpeeds(-5, 0, 0));}),
            new WaitCommand(3),
            new InstantCommand(() -> {SwerveSubsystem.getInstance().driveFieldOriented(new ChassisSpeeds(0, -5, 0));}),
            new WaitCommand(3),
            new InstantCommand(() -> {SwerveSubsystem.getInstance().driveFieldOriented(new ChassisSpeeds(0, 0, 2 * Math.PI));}),
            new WaitCommand(2),
            new InstantCommand(() -> {SwerveSubsystem.getInstance().driveFieldOriented(new ChassisSpeeds(0, 0, -2 * Math.PI));}),
            new WaitCommand(2),
            new InstantCommand(() -> {SwerveSubsystem.getInstance().driveFieldOriented(new ChassisSpeeds(0, 0, 0));}),
            new Collect(),
            new PrepAmp(),
            new WaitCommand(2),
            new TheBigYeet(),
            new WaitCommand(0.1),
            new Collect(),
            new PrepSpeaker(),
            new WaitCommand(0.1),
            new Collect(),
            new HalfClimb(),
            new WaitCommand(1),
            new SimpleClimb(),
            new WaitCommand(1),
            new PrepClimb(),
            new WaitCommand(2),
            new Layup()
        );
    }    
}
