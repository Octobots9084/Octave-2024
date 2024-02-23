package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstantForce;
import frc.robot.commands.arm.ShooterTrackSpeedInstantForce;
import frc.robot.constants.ShooterSpeeds;

public class Panic extends ParallelCommandGroup {
    public Panic() {
        addCommands(
                new ShooterFlywheelSpeedInstantForce(ShooterSpeeds.PANIC),
                new ShooterTrackSpeedInstantForce(ShooterSpeeds.PANIC));
    }
}
