package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.util.MathUtil;

public class ShooterFlywheelSpeedToleranceMeters extends Command {
    ShooterSpeeds shooterSpeeds;
    ShooterFlywheel flywheel;

    public ShooterFlywheelSpeedToleranceMeters(ShooterSpeeds shooterSpeeds) {
        this.shooterSpeeds = shooterSpeeds;
        flywheel = ShooterFlywheel.getInstance();
    }

    @Override
    public void initialize() {
        flywheel.setFlyWheelSpeedMeters(shooterSpeeds.flywheels);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(flywheel.getFlywheelSpeedMeters(), shooterSpeeds.flywheels,
                Constants.Arm.SHOOTER_FLYWHEEL_TOLERANCE_METERS);
    }
}