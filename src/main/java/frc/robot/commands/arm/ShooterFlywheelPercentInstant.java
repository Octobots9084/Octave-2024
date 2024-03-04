package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterFlywheel;

public class ShooterFlywheelPercentInstant extends InstantCommand {

    double percent;
    ShooterFlywheel flywheel;

    public ShooterFlywheelPercentInstant(double percent) {
        this.percent = percent;
        flywheel = ShooterFlywheel.getInstance();

    }

    @Override
    public void initialize() {
        flywheel.setFlywheelPercent(percent);
    }
}