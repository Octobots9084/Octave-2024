package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterTrack;

public class ShooterTrackSpeedInstant extends InstantCommand{

    ShooterSpeeds shooterSpeeds;
    ShooterTrack track;

    public ShooterTrackSpeedInstant(ShooterSpeeds shooterSpeeds) {
        this.shooterSpeeds = shooterSpeeds;
        track = ShooterTrack.getInstance();
        super.addRequirements(track);
    }

    @Override 
    public void initialize() {
        track.setSpeed(shooterSpeeds);
    }
}