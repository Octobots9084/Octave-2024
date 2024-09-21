package frc.robot.commands.complex.collect;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class TriggerLightsOnFirstSensors extends Command {
    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("IntakeTrackAnalogDigital", IntakeTrack.getInstance().getAnalogDigital());
        SmartDashboard.putBoolean("IntakeTrackSensor1", !IntakeTrack.getInstance().getSensor());
        SmartDashboard.putBoolean("IntakeTrackSensor2", !IntakeTrack.getInstance().getSensor2());
        SmartDashboard.putBoolean("IntakeRoller", !IntakeRoller.getInstance().getSensor());

        return !IntakeTrack.getInstance().getSensor() ||
                !IntakeTrack.getInstance().getSensor2();
    }

    @Override
    public void end(boolean interrupted) {
        Light.getInstance().setAnimation(Animations.INTAKE_STAGE_1);
    }
}
