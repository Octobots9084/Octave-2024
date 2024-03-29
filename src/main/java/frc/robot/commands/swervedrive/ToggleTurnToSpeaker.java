package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.MathUtil;

public class ToggleTurnToSpeaker extends Command {

    double startToleranceInTime;
    boolean startedInTolerance = false;

    @Override
    public void initialize() {
        if (!SwerveSubsystem.getInstance().targetAngleEnabled) {
            double target = 0;
            if (!Constants.isBlueAlliance) {
                target = Math.PI;
            }
            SwerveSubsystem.getInstance().targetAngle = new Rotation2d(target);
        }

        SwerveSubsystem.getInstance().targetAngleEnabled = !SwerveSubsystem.getInstance().targetAngleEnabled;
        System.out.println("Rotation command started: " + this.getName());


    }

    @Override
    public void execute() {
        if (MathUtil.isWithinTolerance(SwerveSubsystem.getInstance().getHeading().getRadians(), SwerveSubsystem.getInstance().targetAngle.getRadians(), Constants.Drivebase.TURN_TO_ANGLE_TOLERANCE)) {
            if (!startedInTolerance) {
                startToleranceInTime = Timer.getFPGATimestamp();
                // SmartDashboard.putBoolean("InTolerance", true);
                startedInTolerance = true;
            }
            
        } else {
            startToleranceInTime = Timer.getFPGATimestamp() + 10000;
            // SmartDashboard.putBoolean("InTolerance", false);
            startedInTolerance = false;
        }
    }

     @Override
    public boolean isFinished() {
        return startToleranceInTime + Constants.Drivebase.TURN_TO_ANGLE_TIME_TOLERANCE < Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().targetAngleEnabled = false;
    }
}
