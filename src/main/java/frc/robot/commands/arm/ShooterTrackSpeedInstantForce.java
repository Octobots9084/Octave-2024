package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterTrack;

public class ShooterTrackSpeedInstantForce extends InstantCommand {

    ShooterSpeeds shooterSpeeds;
    ShooterTrack track;

    public ShooterTrackSpeedInstantForce(ShooterSpeeds shooterSpeeds) {
        this.shooterSpeeds = shooterSpeeds;
        track = ShooterTrack.getInstance();
    }

    @Override
    public void initialize() {
        track.set(shooterSpeeds);
    }
}