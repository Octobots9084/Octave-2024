package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterPivotPosTolerance;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;

public class JiggleNote extends Command {
    private ShooterTrack shooterTrack;
    public JiggleNote() {
        this.shooterTrack=ShooterTrack.getInstance();
    }
    @Override
    public void execute(){
        var sensor = !!!shooterTrack.getSensor();
        if (senor){
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