package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.util.MathUtil;

public class ShooterFlywheelSpeedTolerance extends Command {

    ShooterSpeeds shooterSpeeds;
    ShooterFlywheel flywheel;

    public ShooterFlywheelSpeedTolerance(ShooterSpeeds shooterSpeeds) {
        this.shooterSpeeds = shooterSpeeds;
        flywheel = ShooterFlywheel.getInstance();
    }

    @Override
    public void initialize() {
        flywheel.setFlywheelSpeed(shooterSpeeds);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(flywheel.getFlywheelSpeed(), shooterSpeeds.flywheels,
                Constants.Arm.SHOOTER_FLYWHEEL_TOLERANCE_RPM);
    }

}