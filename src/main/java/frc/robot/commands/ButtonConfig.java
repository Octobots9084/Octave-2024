package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.complex.Collect;
import frc.robot.commands.complex.Driveby;
import frc.robot.commands.complex.Panic;
import frc.robot.commands.complex.PrepAmp;
import frc.robot.commands.complex.PrepClimb;
import frc.robot.commands.complex.PrepSpeaker;
import frc.robot.commands.complex.SimpleClimb;
import frc.robot.commands.complex.TheBigYeet;
import frc.robot.commands.swervedrive.ToggleTurnTo180;
import frc.robot.commands.swervedrive.ToggleTurnToAmp;
import frc.robot.commands.swervedrive.ToggleTurnToSource;
import frc.robot.commands.swervedrive.ToggleTurnToSpeaker;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ButtonConfig {
    CommandJoystick driverLeft = new CommandJoystick(Constants.OperatorConstants.DRIVER_LEFT);
    CommandJoystick driverRight = new CommandJoystick(Constants.OperatorConstants.DRIVER_RIGHT);
    CommandJoystick driverButtons = new CommandJoystick(Constants.OperatorConstants.DRIVER_BUTTONS);
    CommandJoystick coDriverLeft = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_LEFT);
    CommandJoystick coDriverRight = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_RIGHT);
    CommandJoystick coDriverButtons = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_BUTTONS);

    public void initTeleop() {
        driverLeft.button(1).onTrue(new SequentialCommandGroup(new PrepAmp(), new ToggleTurnToAmp()));
        driverLeft.button(2).whileTrue(new Driveby());

        driverRight.button(1).onTrue(new TheBigYeet());
        driverRight.button(2).onTrue(new PrepSpeaker());

        driverButtons.button(1).onTrue(new ToggleTurnToSpeaker());
        driverButtons.button(2).onTrue(new ToggleTurnTo180());
        driverButtons.button(3).onTrue(new ToggleTurnToSource());
        driverButtons.button(4).onTrue(new Collect());
        driverButtons.button(5).onTrue(new ToggleTurnToAmp());
        driverButtons.button(6).onTrue(new InstantCommand(SwerveSubsystem.getInstance()::zeroGyro));
        driverButtons.button(11).onTrue(new Panic());
        driverButtons.button(12).onTrue(new CancelAllCommands());

        coDriverButtons.button(1).onTrue(new Collect());
        coDriverButtons.button(7).onTrue(new PrepClimb());
        coDriverButtons.button(8).onTrue(new SimpleClimb());
        coDriverButtons.button(9).onTrue(new ShooterElevatorPosInstant(ArmPositions.TRAP));
        // coDriverButtons.button(11).onTrue(new Dunk());
        // coDriverButtons.button(12).onTrue(new Undunk());

        coDriverButtons.button(12).onTrue(new ShooterFlywheelSpeedInstant(ShooterSpeeds.SPEAKER));

    }
}