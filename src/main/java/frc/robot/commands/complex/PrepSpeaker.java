package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class PrepSpeaker extends InstantCommand {
    private ShooterPivot shooterPivot;
    private ShooterElevator shooterElevator;
    private ShooterFlywheel shooterFlywheel;

    public PrepSpeaker(){
        this.shooterPivot = ShooterPivot.getInstance();
        this.shooterFlywheel = ShooterFlywheel.getInstance();
        this.shooterElevator = ShooterElevator.getInstance();
    }

    @Override
    public void initialize() {
        if (!shooterPivot.notSoFastEggman) {
            shooterElevator.setPosition(ArmPositions.SPEAKER_SHOT);
            shooterPivot.setPosition(ArmPositions.SPEAKER_SHOT);
            shooterFlywheel.setFlyWheelSpeedMeters(ShooterSpeeds.SPEAKER.flywheels);
            Light.getInstance().setAnimation(Animations.SHOT_SPECIAL);
            System.out.println("Prep speaker run successfully.");

        }
        else {
            System.out.println("Prep speaker blocked by pivot lock, try again.");

        }
        
    }
}
