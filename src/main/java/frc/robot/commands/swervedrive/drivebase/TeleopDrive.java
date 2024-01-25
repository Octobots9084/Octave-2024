// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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
  private final SwerveController controller;
  private double targetAngle = 0;
  private boolean initalTargetAngleSet = false;

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
    this.controller = swerve.getSwerveController();
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
    double xVelocity = Math.signum(vX.getAsDouble()) * Math.pow(vX.getAsDouble(), 2);
    double yVelocity = Math.signum(vY.getAsDouble()) * Math.pow(vY.getAsDouble(), 2);
    double angVelocity = Math.signum(omega.getAsDouble()) * Math.pow(omega.getAsDouble(), 2);
    
    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);

    // Drive using raw values.
    if (swerve.getShootingRequestActive()){
      ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), swerve.getShootingRequest());
      if (desiredSpeeds.omegaRadiansPerSecond>2*controller.config.maxAngularVelocity) {
        desiredSpeeds.omegaRadiansPerSecond = 2*controller.config.maxAngularVelocity;
      }
      swerve.drive(new Translation2d(xVelocity * SwerveSubsystem.MAXIMUM_SPEED, yVelocity * SwerveSubsystem.MAXIMUM_SPEED),
        desiredSpeeds.omegaRadiansPerSecond,
        driveMode.getAsBoolean());
    } else if(angVelocity!=0) {
      swerve.drive(new Translation2d(xVelocity * SwerveSubsystem.MAXIMUM_SPEED, yVelocity * SwerveSubsystem.MAXIMUM_SPEED),
        angVelocity * controller.config.maxAngularVelocity,
        driveMode.getAsBoolean());
      initalTargetAngleSet = false;
        
    } else {
      if (!initalTargetAngleSet) {
        targetAngle = swerve.getPose().getRotation().getRadians();
        initalTargetAngleSet = true;
      }
      
      ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
        new Rotation2d(targetAngle));
      if (desiredSpeeds.omegaRadiansPerSecond>2*controller.config.maxAngularVelocity) {
        desiredSpeeds.omegaRadiansPerSecond = 2*controller.config.maxAngularVelocity;
      }
      swerve.drive(new Translation2d(xVelocity * SwerveSubsystem.MAXIMUM_SPEED, yVelocity * SwerveSubsystem.MAXIMUM_SPEED),
        desiredSpeeds.omegaRadiansPerSecond,
        driveMode.getAsBoolean());
      SmartDashboard.putNumber("Target Angle", targetAngle);
      SmartDashboard.putNumber("Omega", desiredSpeeds.omegaRadiansPerSecond);
    }
    
  }

}
