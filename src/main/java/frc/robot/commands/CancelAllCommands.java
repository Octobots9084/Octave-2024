package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;

public class CancelAllCommands extends InstantCommand {

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        ShooterPivot.getInstance().notSoFastEggman = false;
        Light.getInstance().setAnimation(Animations.DEFAULT);
        ShooterTrack.getInstance().set(ShooterSpeeds.STOP);
        IntakeTrack.getInstance().set(ShooterSpeeds.STOP);
        IntakeRoller.getInstance().set(IntakeSpeeds.STOP);
    }
}
