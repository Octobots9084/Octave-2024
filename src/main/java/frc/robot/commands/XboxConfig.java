package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstant;
import frc.robot.commands.climb.ClimbPosTolerance;
import frc.robot.commands.complex.Collect;
import frc.robot.commands.complex.Driveby;
import frc.robot.commands.complex.Dunk;
import frc.robot.commands.complex.Layup;
import frc.robot.commands.complex.Panic;
import frc.robot.commands.complex.PrepAmp;
import frc.robot.commands.complex.PrepClimb;
import frc.robot.commands.complex.PrepSpeaker;
import frc.robot.commands.complex.SimpleClimb;
import frc.robot.commands.complex.TheBigYeet;
import frc.robot.commands.complex.Undunk;
import frc.robot.commands.intake.IntakeRollerSpeedInstant;
import frc.robot.commands.intake.IntakeTrackSpeedInstant;
import frc.robot.commands.swervedrive.ToggleTurnTo180;
import frc.robot.commands.swervedrive.ToggleTurnToAmp;
import frc.robot.commands.swervedrive.ToggleTurnToSource;
import frc.robot.commands.swervedrive.ToggleTurnToSpeaker;
import frc.robot.constants.ClimbPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
public class XboxConfig {
    public static CommandXboxController driver = new CommandXboxController(0);
    XboxController codriver = new XboxController(1);
CommandJoystick driverLeft = new CommandJoystick(Constants.OperatorConstants.DRIVER_LEFT);
    CommandJoystick driverRight = new CommandJoystick(Constants.OperatorConstants.DRIVER_RIGHT);
    CommandJoystick driverButtons = new CommandJoystick(Constants.OperatorConstants.DRIVER_BUTTONS);
    CommandJoystick coDriverLeft = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_LEFT);
    CommandJoystick coDriverRight = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_RIGHT);
    CommandJoystick coDriverButtons = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_BUTTONS);

    public void initTeleop() {
        //b button: source align
        //a button: to 180 deg
        //y button: to 0 deg
        //x button: gyro 0
        //left stick: turn
        //right stick: drive
        //dpad left: panic
        //dpad right: cancel
        //rt: fire
        //rb: intake
        //lt: amp
        //lb: driveby
        driver.button(1).onTrue(new ToggleTurnTo180());
        driver.button(2).onTrue(new ToggleTurnToSource());
        driver.button(3).onTrue(new InstantCommand(SwerveSubsystem.getInstance()::zeroGyro));
        driver.button(4).onTrue(new ToggleTurnToSpeaker());

        driver.button(5).whileTrue(new Driveby());
        driver.button(6).onTrue(new Collect());

        driverRight.button(1).onTrue(new TheBigYeet());
        driverRight.button(2).onTrue(new PrepSpeaker());

        driver.rightTrigger().onTrue(new TheBigYeet());
        driver.leftTrigger().onTrue(new SequentialCommandGroup(new PrepAmp(), new ToggleTurnToAmp()));

        driver.povLeft().onTrue(new Panic());
        driver.povRight().onTrue(new CancelAllCommands());
    }
}
