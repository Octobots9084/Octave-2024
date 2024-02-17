// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ReverseKinematics;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.MathUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDrive extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final DoubleSupplier omega;
  private final BooleanSupplier driveMode;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
      BooleanSupplier driveMode) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (swerve.getShootingRequestActive()) {
      swerve.targetAngleEnabled = true;
      swerve.targetAngle = new Rotation2d(ReverseKinematics.calcRobotAngle(
          ReverseKinematics.convert2dCoords(swerve.getPose()),
          ReverseKinematics.convertSpeed(ReverseKinematics.convert2dCoords(swerve.getPose()),
              swerve.getRobotVelocity()),
          ShooterSpeeds.DRIVE_BY.flywheels));
    } else {
      swerve.targetAngleEnabled = false;
    }

    if (swerve.targetAngleEnabled) {
      swerve.drive(
          new Translation2d(vX.getAsDouble() * SwerveSubsystem.MAXIMUM_SPEED,
              vY.getAsDouble() * SwerveSubsystem.MAXIMUM_SPEED),
          swerve.targetAngleController.calculate(swerve.getHeading().getRadians(), swerve.targetAngle.getRadians()),
          driveMode.getAsBoolean());
    } else {
      swerve.drive(
          new Translation2d(vX.getAsDouble() * SwerveSubsystem.MAXIMUM_SPEED,
              vY.getAsDouble() * SwerveSubsystem.MAXIMUM_SPEED),
          omega.getAsDouble() * 6 * Math.PI,
          driveMode.getAsBoolean());
    }

  }

}
