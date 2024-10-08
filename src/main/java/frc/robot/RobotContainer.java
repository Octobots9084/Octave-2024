// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.complex.Collect;
import frc.robot.commands.complex.CollectAuto;
import frc.robot.commands.complex.CollectDriveby;
import frc.robot.commands.complex.CollectDrivebySafely;
import frc.robot.commands.complex.DrivebyAuto;
import frc.robot.commands.complex.DrivebyAutoHigher;
import frc.robot.commands.complex.DrivebyAutoSniper;
import frc.robot.commands.complex.InitalSpeakerAuto;
import frc.robot.commands.complex.InitalSpeakerAutoFast;
import frc.robot.commands.complex.SpeakerAutoFast;
import frc.robot.commands.complex.TheBigYeet;
import frc.robot.commands.complex.TheBigYeetAuto;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.PieceVision;
import frc.robot.subsystems.vision.VisionEstimation;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems and commands are defined here...
        // Replace with CommandPS4Controller or CommandJoystick if needed
        CommandJoystick driverLeft = new CommandJoystick(Constants.OperatorConstants.DRIVER_LEFT);
        CommandJoystick driverRight = new CommandJoystick(Constants.OperatorConstants.DRIVER_RIGHT);
        CommandJoystick driverButtons = new CommandJoystick(Constants.OperatorConstants.DRIVER_BUTTONS);
        CommandJoystick coDriverLeft = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_LEFT);
        CommandJoystick coDriverRight = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_RIGHT);
        CommandJoystick coDriverButtons = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_BUTTONS);

        private final VisionEstimation visionEstimation = VisionEstimation.getInstance();
        private final PieceVision pieceVision = PieceVision.getInstance();
        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings

                TeleopDrive closedFieldRel = new TeleopDrive(
                                SwerveSubsystem.getInstance(),
                                () -> MathUtil.applyDeadband(-driverLeft.getRawAxis(1),
                                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(-driverLeft.getRawAxis(0),
                                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> MathUtil.applyDeadband(-driverRight.getRawAxis(0),
                                                OperatorConstants.RIGHT_X_DEADBAND),
                                () -> true);

                SwerveSubsystem.getInstance().setDefaultCommand(
                                !RobotBase.isSimulation() ? closedFieldRel : closedFieldRel);
                try {
                        BooleanSupplier shooterSensorTrue = () -> !ShooterTrack.getInstance().getSensor();
                        NamedCommands.registerCommand("SpeakerAuto", new SpeakerAutoFast());
                        NamedCommands.registerCommand("SpeakerAutoInital", new InitalSpeakerAutoFast());
                        NamedCommands.registerCommand("SpeakerAutoSlow", new InitalSpeakerAuto());

                        NamedCommands.registerCommand("CollectDrivebyMF", new CollectDriveby());
                        NamedCommands.registerCommand("CollectDrivebyMFSafely", new CollectDrivebySafely());

                        NamedCommands.registerCommand("Collect", new Collect().withTimeout(9));
                        NamedCommands.registerCommand("Snipe", new DrivebyAutoSniper(false));

                        NamedCommands.registerCommand("QuickDraw",
                                        new DrivebyAuto(false).withTimeout(2).andThen(new WaitCommand(0.1)));

                        NamedCommands.registerCommand("Shoot",
                                        new DrivebyAuto(false).withTimeout(1.5).andThen(new TheBigYeetAuto()));
                        NamedCommands.registerCommand("ShootSafelyHigher",
                                        new ConditionalCommand(
                                                        new DrivebyAutoHigher(false).withTimeout(1.5)
                                                                        .andThen(new TheBigYeetAuto()),
                                                        new InstantCommand(), shooterSensorTrue));
                        NamedCommands.registerCommand("ShootSafely",
                                        new ConditionalCommand(
                                                        new DrivebyAuto(false).withTimeout(1.5)
                                                                        .andThen(new TheBigYeetAuto()),
                                                        new InstantCommand(), shooterSensorTrue));
                        NamedCommands.registerCommand("StopShooterTrack", new InstantCommand(() -> {
                                ShooterTrack.getInstance().set(ShooterSpeeds.STOP);
                        }));

                        NamedCommands.registerCommand("SpinUpFlywheels", new InstantCommand(() -> {
                                ShooterFlywheel.getInstance().setFlyWheelSpeedMeters(-20);
                                ShooterPivot.getInstance().setPosition(ArmPositions.HANDOFF_AND_DEFAULT_SHOT);
                        }));
                        NamedCommands.registerCommand("SpinUpFlywheelsFast", new InstantCommand(() -> {
                                ShooterFlywheel.getInstance().setFlyWheelSpeedMeters(-500);
                                ShooterPivot.getInstance().setPosition(ArmPositions.HANDOFF_AND_DEFAULT_SHOT);
                        }));

                        NamedCommands.registerCommand("FlywheelsCurrentFast", new InstantCommand(() -> {
                                ShooterFlywheel.getInstance().setFlywheelsCurrentFast();

                        }));

                        NamedCommands.registerCommand("FlywheelsCurrentNormal", new InstantCommand(() -> {
                                ShooterFlywheel.getInstance().setFlywheelsCurrentNormal();
                        }));
                        AutoBuilder.configureHolonomic(
                                        SwerveSubsystem.getInstance()::getPose, // Robot pose supplier
                                        SwerveSubsystem.getInstance()::resetOdometry, // Method to reset odometry (will
                                        // be
                                        // called if your auto
                                        // has a starting pose)
                                        SwerveSubsystem.getInstance()::getRobotVelocity, // ChassisSpeeds supplier. MUST
                                        // BE
                                        // ROBOT RELATIVE
                                        SwerveSubsystem.getInstance()::setChassisSpeeds, // Method that will drive the
                                        // robot
                                        // given ROBOT
                                        // RELATIVE ChassisSpeeds
                                        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
                                                        // likely live
                                                        // in your
                                                        // Constants class
                                                        Constants.Auton.TRANSLATION_PID,
                                                        // Translation PID constants
                                                        Constants.Auton.ANGLE_AUTO_PID,
                                                        // Rotation PID constants
                                                        Constants.Auton.MAX_MODULE_SPEED,
                                                        // Max module speed, in m/s
                                                        SwerveSubsystem.getInstance()
                                                                        .getSwerveDrive().swerveDriveConfiguration
                                                                        .getDriveBaseRadiusMeters(),
                                                        // Drive base radius in meters. Distance from robot center to
                                                        // furthest
                                                        // module.
                                                        new ReplanningConfig()
                                        // Default path replanning config. See the API for the options here
                                        ),
                                        () -> {
                                                // Boolean supplier that controls when the path will be mirrored for the
                                                // red
                                                // alliance
                                                // This will flip the path being followed to the red side of the field.
                                                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                                                return !Constants.isBlueAlliance;
                                        },
                                        SwerveSubsystem.getInstance() // Reference to this subsystem to set requirements
                        );

                } catch (Exception err) {
                        System.out.println(err);
                }
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Competition", autoChooser);
                SwerveSubsystem.getInstance();

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setDriveMode() {
                // SwerveSubsystem.getInstance().setDefaultCommand();
        }

        // public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        // }

        public VisionEstimation getVisionEstimation() {
                return visionEstimation;
        }

}
