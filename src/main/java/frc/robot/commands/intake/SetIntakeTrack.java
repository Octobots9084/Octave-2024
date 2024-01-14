package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.subsystems.IntakeTrack;

public class SetIntakeTrack extends Command{
    //created by Olorin

    IntakeSpeeds intakeSpeed;
    IntakeTrack track;

    public SetIntakeTrack(IntakeSpeeds intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        track = IntakeTrack.getInstance();
        super.addRequirements(track);
    }

    @Override 
    public void initialize() {
        track.setSpeed(intakeSpeed.track);
    }
}