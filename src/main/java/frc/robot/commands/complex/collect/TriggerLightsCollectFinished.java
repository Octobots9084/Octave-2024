package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class TriggerLightsCollectFinished extends InstantCommand {
    @Override
    public void initialize() {
        Light.getInstance().setAnimation(Animations.SHOT_READY);

    }

}