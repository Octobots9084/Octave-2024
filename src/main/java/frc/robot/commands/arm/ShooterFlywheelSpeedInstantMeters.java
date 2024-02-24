package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterFlywheel;

public class ShooterFlywheelSpeedInstantMeters extends InstantCommand {

    ShooterSpeeds shooterSpeeds;
    ShooterFlywheel flywheel;

    public ShooterFlywheelSpeedInstantMeters(ShooterSpeeds shooterSpeeds) {
        this.shooterSpeeds = shooterSpeeds;
        flywheel = ShooterFlywheel.getInstance();
        super.addRequirements(flywheel);

    }

    @Override
    public void initialize() {
        flywheel.setFlyWheelSpeedMeters(shooterSpeeds.flywheels);
    }
}