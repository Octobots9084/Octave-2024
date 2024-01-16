package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.constants.IntakeSpeeds;

public class SetIntakeRoller extends Command{
    IntakeRoller intakeroller;
    IntakeSpeeds speed; 
    public SetIntakeRoller(IntakeSpeeds givein) {
        this.speed = givein;
        intakeroller = IntakeRoller.getInstance();
        super.addRequirements(intakeroller);
    }

    @Override 
    public void initialize() {
       intakeroller.setVelocity(speed);
    }
}
