package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.subsystems.IntakeTrack;

public class IntakeTrackSpeedInstant extends InstantCommand {
    // created by Olorin

    IntakeSpeeds intakeSpeed;
    IntakeTrack track;

    public IntakeTrackSpeedInstant(IntakeSpeeds intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        track = IntakeTrack.getInstance();
    }

    @Override
    public void initialize() {
        track.set(intakeSpeed.track);
    }
}