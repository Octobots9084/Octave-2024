package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstantForce;
import frc.robot.commands.arm.ShooterFlywheelSpeedInstantMeters;
import frc.robot.commands.arm.ShooterTrackSpeedInstant;
import frc.robot.commands.climb.ClimbZero;
import frc.robot.commands.complex.Collect;
import frc.robot.commands.complex.Driveby;
import frc.robot.commands.complex.FerryShot;
import frc.robot.commands.complex.HalfClimb;
import frc.robot.commands.complex.Layup;
import frc.robot.commands.complex.Panic;
import frc.robot.commands.complex.PrepAmp;
import frc.robot.commands.complex.PrepClimb;
import frc.robot.commands.complex.PrepSpeaker;
import frc.robot.commands.complex.SimpleClimb;
import frc.robot.commands.complex.TheBigYeet;
import frc.robot.commands.complex.Undunk;
import frc.robot.commands.swervedrive.ToggleTurnTo180;
import frc.robot.commands.swervedrive.ToggleTurnToAmp;
import frc.robot.commands.swervedrive.ToggleTurnToSource;
import frc.robot.commands.swervedrive.ToggleTurnToSpeaker;
import frc.robot.constants.ArmPositions;
import frc.robot.commands.vision.AmpAlign;
import frc.robot.constants.ClimbPositions;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.robot.ControlMap;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ButtonConfig {
    CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
    CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
    CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
    CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
    CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
    CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

    public void initTeleop() {
        // driverLeft.button(1).whileTrue(new TagAlign());
        driverLeft.button(1).whileTrue(new ParallelCommandGroup(new PrepAmp(), new AmpAlign()));
        driverLeft.button(2).whileTrue(new Driveby());

        driverRight.button(1).onTrue(new TheBigYeet());
        driverRight.button(2).onTrue(new FerryShot());

        driverButtons.button(1).onTrue(new ToggleTurnToSpeaker());
        driverButtons.button(2).onTrue(new ToggleTurnTo180());
        driverButtons.button(3).onTrue(new ToggleTurnToSource());
        driverButtons.button(4).onTrue(new Collect());
        driverButtons.button(5).onTrue(new ShooterTrackSpeedInstant(ShooterSpeeds.REVERSE_TRACK));
        driverButtons.button(6).onTrue(new InstantCommand(SwerveSubsystem.getInstance()::zeroGyro));
        driverButtons.button(7).onTrue(new PrepSpeaker());
        driverButtons.button(11).onTrue(new Panic());
        driverButtons.button(9).onTrue(new CancelAllCommands());
        driverButtons.button(8).onTrue(new InstantCommand(() -> {
            IntakeTrack.getInstance().set(IntakeSpeeds.PANIC);
            IntakeRoller.getInstance().set(IntakeSpeeds.PANIC);
        }));
        driverButtons.button(12).onTrue(new ShooterFlywheelSpeedInstantMeters(ShooterSpeeds.DRIVE_BY));

        coDriverButtons.button(1).onTrue(new PrepClimb());
        coDriverButtons.button(2).onTrue(new HalfClimb());
        coDriverButtons.button(3).onTrue(new SimpleClimb());
        coDriverButtons.button(4).onTrue(new Layup());
        coDriverButtons.button(5).onTrue(new Undunk());
        coDriverButtons.button(6).whileTrue(new ClimbZero());
        coDriverButtons.button(7).onTrue(new Collect());
        coDriverButtons.button(8).onTrue(new InstantCommand()); // climb align
        coDriverButtons.button(9).whileTrue(new SequentialCommandGroup(new InstantCommand(() -> {
            ShooterElevator.getInstance().setPosition(ArmPositions.AMP);
        }), new Driveby()));
        coDriverButtons.button(10).onTrue(
                new SequentialCommandGroup(new InstantCommand(() -> {
                    ShooterElevator.getInstance().setPosition(ArmPositions.PREP_TRAP);
                }),
                        new InstantCommand(() -> {
                            ShooterPivot.getInstance().setPosition(ArmPositions.PREP_TRAP);
                        }),
                        new InstantCommand(() -> {
                            ShooterFlywheel.getInstance().setFlywheelSpeed(0);
                        })));
        coDriverButtons.button(11).onTrue(new Panic());
        coDriverButtons.button(12).onTrue(new CancelAllCommands());

        // coDriverButtons.button(4).onTrue(new SequentialCommandGroup(new
        // IntakeTrackSpeedInstant(IntakeSpeeds.COLLECT), new
        // IntakeRollerSpeedInstant(IntakeSpeeds.COLLECT)));
        // coDriverButtons.button(7).onTrue(new PrepClimb());
        // coDriverButtons.button(8).onTrue(new ClimbPosTolerance(ClimbPositions.MID));
        // coDriverButtons.button(9).onTrue(new SimpleClimb());
        // coDriverButtons.button(10).onTrue(new Layup());
        // coDriverButtons.button(11).onTrue(new Undunk());
        // coDriverButtons.button(12).onTrue(new CancelAllCommands());

        // coDriverButtons.button(12).onTrue(new
        // ShooterFlywheelSpeedInstant(ShooterSpeeds.SPEAKER));

    }
}
