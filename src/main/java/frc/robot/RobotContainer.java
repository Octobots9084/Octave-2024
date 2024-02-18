// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.PathfindingTest;
import frc.robot.commands.swervedrive.auto.TestingPaths;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionEstimation;
import swervelib.imu.NavXSwerve;

import java.io.File;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

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

    // CommandJoystick driverController = new

    private final VisionEstimation visionEstimation = new VisionEstimation();

    private final SendableChooser<Command> autoChooser;
    
    XboxController controller = new XboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        // AbsoluteDrive closedAbsoluteDrive = new
        // AbsoluteDrive(SwerveSubsystem.getInstance(),
        // // Applies deadbands and inverts controls because joysticks
        // // are back-right positive while robot
        // // controls are front-left positive
        // () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
        // OperatorConstants.LEFT_Y_DEADBAND),
        // () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
        // OperatorConstants.LEFT_X_DEADBAND),
        // () -> -driverXbox.getRightX(),
        // () -> -driverXbox.getRightY());

        // AbsoluteFieldDrive closedFieldAbsoluteDrive = new
        // AbsoluteFieldDrive(SwerveSubsystem.getInstance(),
        // () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
        // OperatorConstants.LEFT_Y_DEADBAND),
        // () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
        // OperatorConstants.LEFT_X_DEADBAND),
        // () -> driverXbox.getRawAxis(2));

        // TeleopDrive simClosedFieldRel = new
        // TeleopDrive(SwerveSubsystem.getInstance(),
        // () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
        // OperatorConstants.LEFT_Y_DEADBAND),
        // () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
        // OperatorConstants.LEFT_X_DEADBAND),
        // () -> driverXbox.getRawAxis(2), () -> true);
        TeleopDrive closedFieldRel = new TeleopDrive(
                SwerveSubsystem.getInstance(),
                () -> MathUtil.applyDeadband(-driverLeft.getRawAxis(1),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverLeft.getRawAxis(0),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(-driverRight.getRawAxis(0), OperatorConstants.RIGHT_X_DEADBAND),
                () -> true);
        
        // TeleopDrive closedFieldRel = new TeleopDrive(
        //         SwerveSubsystem.getInstance(),
        //         () -> MathUtil.applyDeadband(-controller.getLeftX(),
        //                 OperatorConstants.LEFT_Y_DEADBAND),
        //         () -> MathUtil.applyDeadband(controller.getLeftY(),
        //                 OperatorConstants.LEFT_X_DEADBAND),
        //         () -> MathUtil.applyDeadband(-controller.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        //         () -> true);

        SwerveSubsystem.getInstance().setDefaultCommand(
                !RobotBase.isSimulation() ? closedFieldRel : closedFieldRel);
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        driverLeft.button(1).onTrue((new InstantCommand(SwerveSubsystem.getInstance()::zeroGyro)));
        driverLeft.button(2).onTrue((new InstantCommand(() -> {
            SmartDashboard.putNumber("button press", 0);
            // Not safe type casting, could break but should be obvious
            NavXSwerve navx = (NavXSwerve) SwerveSubsystem.getInstance().getSwerveDrive().imu;
            AHRS gyro = (AHRS) navx.getIMU();
            gyro.reset();

        })));

        // driverRight.button(1).onTrue(PathfindingTest.getTest());

        // if(controller.getAButtonPressed()){
        //     new InstantCommand(SwerveSubsystem.getInstance()::zeroGyro);
        // }
        // if(controller.getBButtonPressed()){
        //         new InstantCommand(() -> {
        //         SmartDashboard.putNumber("button press", 0);
        //         // Not safe type casting, could break but should be obvious
        //         NavXSwerve navx = (NavXSwerve) SwerveSubsystem.getInstance().getSwerveDrive().imu;
        //         AHRS gyro = (AHRS) navx.getIMU();
        //         gyro.reset();
        //         });
        // }
    }

    public void setDriveMode() {
        // SwerveSubsystem.getInstance().setDefaultCommand();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        SwerveSubsystem.getInstance().setMotorBrake(brake);
    }

    public VisionEstimation getVisionEstimation() {
        return visionEstimation;
    }
}
