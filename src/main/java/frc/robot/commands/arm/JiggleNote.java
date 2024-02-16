package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterTrack;

public class JiggleNote extends Command {
    private ShooterTrack shooterTrack;
    public JiggleNote() {
        this.shooterTrack=ShooterTrack.getInstance();
    }
    @Override
    public void execute(){
        var sensor = !shooterTrack.getSensor();
        if (sensor){
            shooterTrack.set(ShooterSpeeds.JIGGLE_BACKWARD);
        }
        else {
            shooterTrack.set(ShooterSpeeds.JIGGLE_FORWARD);
        }
    }

    @Override 
    public void end(boolean interrupted){
        shooterTrack.set(ShooterSpeeds.STOP);
    }
}