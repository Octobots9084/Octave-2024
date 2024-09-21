package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterTrack;

public class JiggleNote extends Command {
    private ShooterTrack shooterTrack;
    private double startTime;
    private double length;

    public JiggleNote(double length) {
        this.shooterTrack = ShooterTrack.getInstance();
        this.length = length;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        if (shooterTrack.currentlyShooting) {
            return true;
        }
        boolean sensor = !shooterTrack.getSensor();
        if (sensor && Timer.getFPGATimestamp() - startTime > length) {
            shooterTrack.set(ShooterSpeeds.STOP);
            return true;
        } else if (sensor) {
            shooterTrack.setVoltage(ShooterSpeeds.JIGGLE_BACKWARD.track);
        } else {
            shooterTrack.setVoltage(ShooterSpeeds.JIGGLE_FORWARD.track);
        }

        return false;
    }

}