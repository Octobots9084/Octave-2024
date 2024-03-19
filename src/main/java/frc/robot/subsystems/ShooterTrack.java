package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterSpeeds;

public class ShooterTrack extends SubsystemBase {

    public static ShooterTrack shootingRetainer;
    private TalonFX motor1;
    private DigitalInput sensor;
    public boolean pieceGood = false;
    public boolean currentlyShooting = false;

    /*
     * Things this needs to do:
     * 1. needs to be able to run motor when told
     * 2. needs to be able to check if the belt has a Note
     * 3. needs to be able to
     */

    public static ShooterTrack getInstance() {
        if (shootingRetainer == null) {
            shootingRetainer = new ShooterTrack();
        }
        return shootingRetainer;
    }

    public ShooterTrack() {

        motor1 = new TalonFX(15);

        sensor = new DigitalInput(0);
    }

    public void set(double percent) {
        motor1.set(percent);
    }

    public void set(ShooterSpeeds shooterSpeeds) {
        set(shooterSpeeds.track);
    }

    public boolean getSensor() {
        return sensor.get();
    }
}
