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
    public void execute() {
        var sensor = !shooterTrack.getSensor();
        if (sensor) {
            shooterTrack.set(ShooterSpeeds.JIGGLE_BACKWARD);
        } else {
            shooterTrack.set(ShooterSpeeds.JIGGLE_FORWARD);
        }
    }

    @Override
    public boolean isFinished() {
        return (!shooterTrack.getSensor() && Timer.getFPGATimestamp() - startTime > length);
    }

    @Override
    public void end(boolean interrupted) {
        shooterTrack.set(ShooterSpeeds.STOP);
    }
}