// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeTrack;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.ButtonConfig;
import frc.robot.commands.arm.ElevatorManual;
import frc.robot.commands.arm.PivotManual;
import frc.robot.commands.climb.ClimbManual;
import frc.robot.commands.complex.SystemsCheck;
import frc.robot.constants.IntakeSpeeds;
import frc.robot.constants.ShooterSpeeds;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import com.pathplanner.lib.commands.PathfindingCommand;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;
  private double doubleSensorTriggerLength = 0;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        Constants.isBlueAlliance = false;
      }
      if (ally.get() == Alliance.Blue) {
        Constants.isBlueAlliance = true;
      }
    }
    m_robotContainer = new RobotContainer();
    new ButtonConfig().initTeleop();
    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();
    // CommandScheduler.getInstance().setDefaultCommand(Climb.getInstance(), new
    // ClimbManual());
    SwerveSubsystem.getInstance();
    ShooterPivot.getInstance();
    ShooterTrack.getInstance();
    IntakeTrack.getInstance();
    IntakeRoller.getInstance();
    ShooterElevator.getInstance();
    ShooterFlywheel.getInstance();
    Climb.getInstance();
    Light.getInstance().setAnimation(Animations.DEFAULT);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // SmartDashboard.putNumber("thng",
    // ShooterFlywheel.getInstance().getLeftFlywheelSpeed());
    // SmartDashboard.putNumber("thng2",
    // ShooterFlywheel.getInstance().getRightFlywheelSpeed());
    // SmartDashboard.putNumber("elevator position",
    // ShooterElevator.getInstance().getPosition());
    // SmartDashboard.putNumber("desired elevator pos",
    // ShooterElevator.getInstance().getDesiredPosition());
    // SmartDashboard.putNumber("realFlywheelTop",
    // ShooterFlywheel.getInstance().getFlywheelSpeedMeters());
    // SmartDashboard.putNumber("realFlywheelBottom",
    // ShooterFlywheel.getInstance().getAuxiluryFlywheelSpeedMeters());
    SmartDashboard.putBoolean("Shooter track", ShooterTrack.getInstance().getSensor());
    SmartDashboard.putBoolean("Intake track", IntakeTrack.getInstance().getAnalogDigital());
    SmartDashboard.putBoolean("Intake 1", IntakeRoller.getInstance().getSensor());
    SmartDashboard.putBoolean("Intake 2", IntakeTrack.getInstance().getSensor2());
    // SmartDashboard.putNumber("climbele",
    // ShooterElevator.getInstance().getPosition() *
    // ShooterElevator.getInstance().gearing);

    // SmartDashboard.putNumber("realFlywheelBottom",
    // ShooterFlywheel.getInstance().getAuxiluryFlywheelSpeedMeters());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    disabledTimer.reset();
    disabledTimer.start();
    Light.getInstance().setAnimation(Animations.DEFAULT);
  }

  DigitalInput coastSwitch = new DigitalInput(69);

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME)) {
      disabledTimer.stop();
    }
    Climb.getInstance().setCoast(coastSwitch.get());
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        Constants.isBlueAlliance = false;
      }
      if (ally.get() == Alliance.Blue) {
        Constants.isBlueAlliance = true;
      }
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    Climb.getInstance().setCoast(false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Climb.getInstance().setCoast(false);
    m_robotContainer.setDriveMode();
    ShooterFlywheel.getInstance().setFlywheelsCurrentNormal();
    Climb.getInstance().setDefaultCommand(new ClimbManual());
    ShooterPivot.getInstance().setDefaultCommand(new PivotManual());
    ShooterElevator.getInstance().setDefaultCommand(new ElevatorManual());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    checkDoubleNotes();
    // ShooterFlywheel.getInstance().increaseFlywheelSpeed(MathUtil.applyDeadband(5*ControlMap.FLYWHEEL_JOYSTICK.getY(),
    // 0.05));
  }

  public void checkDoubleNotes() {
    SmartDashboard.putBoolean("dn condition 1", !ShooterTrack.getInstance().getSensor());
    SmartDashboard.putBoolean("dn condition 2", !ShooterPivot.getInstance().notSoFastEggman);
    SmartDashboard.putBoolean("dn condition 3", !IntakeRoller.getInstance().getSensor());
    SmartDashboard.putBoolean("dn condition 4", !IntakeTrack.getInstance().getSensor());
    SmartDashboard.putBoolean("dn condition 5", !ShooterPivot.getInstance().notSoFastEggman);
    SmartDashboard.putBoolean("dn condition 6", !IntakeTrack.getInstance().getSensor2());

    if (!ShooterTrack.getInstance().getSensor() && !ShooterPivot.getInstance().notSoFastEggman
        && (!IntakeRoller.getInstance().getSensor()
            || !IntakeTrack.getInstance().getSensor()
            || !IntakeTrack.getInstance().getSensor2())) {
      IntakeRoller.getInstance().set(IntakeSpeeds.PANIC);
      IntakeTrack.getInstance().set(IntakeSpeeds.PANIC);
      Light.getInstance().setAnimation(Animations.CLIMB);
    }

    // if (Timer.getFPGATimestamp() > doubleSensorTriggerLength +
    // Constants.DOUBLE_NOTE_LENGTH) {
    // IntakeTrack.getInstance().set(IntakeSpeeds.PANIC);
    // IntakeRoller.getInstance().set(IntakeSpeeds.PANIC);
    // }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    Climb.getInstance().setCoast(false);
    CommandScheduler.getInstance().schedule(new SystemsCheck());

    try {
      File swerveFile = new File(Filesystem.getDeployDirectory(), "swerve");
      new SwerveParser(swerveFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
