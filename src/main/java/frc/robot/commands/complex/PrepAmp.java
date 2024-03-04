package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;

public class PrepAmp extends InstantCommand {
    private ShooterPivot shooterPivot;
    private ShooterElevator shooterElevator;
    private ShooterFlywheel shooterFlywheel;

    public PrepAmp(){
        this.shooterPivot = ShooterPivot.getInstance();
        this.shooterFlywheel = ShooterFlywheel.getInstance();
        this.shooterElevator = ShooterElevator.getInstance();
    }

    @Override
    public void initialize() {
        if (shooterPivot.notSoFastEggman) {
            return;
        }
        shooterElevator.setPosition(ArmPositions.AMP);
        shooterPivot.setPosition(ArmPositions.AMP);
        shooterFlywheel.setFlyWheelSpeedMeters(ShooterSpeeds.AMP.flywheels);
        Light.getInstance().setAnimation(Animations.SHOT_SPECIAL);
    }
}
