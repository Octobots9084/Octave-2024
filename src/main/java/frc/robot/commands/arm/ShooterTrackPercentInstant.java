package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterTrack;

public class ShooterTrackPercentInstant extends InstantCommand {

    double percent;
    ShooterTrack track;

    public ShooterTrackPercentInstant(double percent) {
        this.percent = percent;
        track = ShooterTrack.getInstance();

    }

    @Override
    public void initialize() {
        track.set(percent);
    }
}