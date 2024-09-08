package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class TriggerLightsOnFirstSensors extends Command {
    @Override
    public boolean isFinished() {
        return !IntakeRoller.getInstance().getSensor() || !IntakeTrack.getInstance().getAnalogDigital()
                || !IntakeTrack.getInstance().getSensor() || !IntakeTrack.getInstance().getSensor2();
    }

    @Override
    public void end(boolean interrupted) {
        Light.getInstance().setAnimation(Animations.JADEN_U_HAVE_A_NOTE);
    }
}
