package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterFlywheel;

public class ButtonConfig {
    public void initTeleop() {
        //ShooterFlywheelSpeedInstant shooter = new ShooterFlywheelSpeedInstant(ShooterSpeeds.IDLE);

        new JoystickButton(ControlMap.DRIVER_BUTTONS, 1).onTrue(new ShooterFlywheelSpeedInstant(ShooterSpeeds.IDLE));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 1).onFalse(new ShooterFlywheelSpeedInstant(ShooterSpeeds.STOP));
    }
}
