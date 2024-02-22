package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ToggleTurnToSpeaker extends InstantCommand {
    @Override
    public void initialize() {
        if (!SwerveSubsystem.getInstance().targetAngleEnabled) {
            SwerveSubsystem.getInstance().targetAngle = new Rotation2d(0);
        }

        SwerveSubsystem.getInstance().targetAngleEnabled = !SwerveSubsystem.getInstance().targetAngleEnabled;
    }
}
