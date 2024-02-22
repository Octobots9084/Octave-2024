package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterFlywheel;

public class ShooterFlywheelSpeedInstantForce extends InstantCommand {

    ShooterSpeeds shooterSpeeds;
    ShooterFlywheel flywheel;

    public ShooterFlywheelSpeedInstantForce(ShooterSpeeds shooterSpeeds) {
        this.shooterSpeeds = shooterSpeeds;
        flywheel = ShooterFlywheel.getInstance();

    }

    @Override
    public void initialize() {
        flywheel.setFlywheelSpeed(shooterSpeeds);
    }
}