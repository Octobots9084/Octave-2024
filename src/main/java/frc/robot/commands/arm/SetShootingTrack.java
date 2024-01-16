package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.subsystems.ShootingTrack;

public class SetShootingTrack extends Command{
    //created by Olorin

    IntakeSpeeds intakeSpeed;
    ShootingTrack track;

    public SetShootingTrack(IntakeSpeeds intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        track = ShootingTrack.getInstance();
        super.addRequirements(track);
    }

    @Override 
    public void initialize() {
        track.setSpeed(intakeSpeed.track);
    }
}