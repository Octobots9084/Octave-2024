package frc.robot.commands.complex;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterElevatorPosTolerance;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.arm.ShooterPivotPosInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;
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
        shooterFlywheel.setFlywheelSpeed(ShooterSpeeds.AMP);
        Light.getInstance().setAnimation(Animations.SHOT_SPECIAL);
    }
}
