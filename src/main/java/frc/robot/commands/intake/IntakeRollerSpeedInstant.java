package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.constants.IntakeSpeeds;

public class IntakeRollerSpeedInstant extends InstantCommand{
    IntakeRoller intakeroller;
    IntakeSpeeds intakeSpeeds; 
    public IntakeRollerSpeedInstant(IntakeSpeeds intakeSpeeds) {
        this.intakeSpeeds = intakeSpeeds;
        intakeroller = IntakeRoller.getInstance();
        super.addRequirements(intakeroller);
    }

    @Override 
    public void initialize() {
       intakeroller.setVelocity(intakeSpeeds);
    }
}
