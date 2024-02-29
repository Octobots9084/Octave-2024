package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.MathUtil;

public class ToggleTurnToSource extends Command {
    @Override
    public void initialize() {
        if (!SwerveSubsystem.getInstance().targetAngleEnabled) {
            SwerveSubsystem.getInstance().targetAngle = new Rotation2d(Math.toRadians(60));
        }

        SwerveSubsystem.getInstance().targetAngleEnabled = !SwerveSubsystem.getInstance().targetAngleEnabled;
    }

     @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(SwerveSubsystem.getInstance().getHeading().getRadians(), SwerveSubsystem.getInstance().targetAngle.getRadians(), Constants.Drivebase.TURN_TO_ANGLE_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().targetAngleEnabled = false;
    }
}
