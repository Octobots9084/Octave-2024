// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ReverseKinematics;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.PieceVision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
      swerve.targetAngle = swerve.getShootingRequest();
    } else if (swerve.getAlignRequestActive()) {
      swerve.targetAngle = swerve.getAlignRequest().getRotation();
    }
    double xSpeed = vX.getAsDouble() * SwerveSubsystem.MAXIMUM_SPEED;
    double ySpeed = vY.getAsDouble() * SwerveSubsystem.MAXIMUM_SPEED;
    if (!Constants.isBlueAlliance) {
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    if (swerve.targetAngleEnabled) {
      double angleSpeed = swerve.targetAngleController.calculate(swerve.getHeading().getRadians(),
          swerve.targetAngle.getRadians());
      if (!swerve.getShootingRequestActive()) {
        angleSpeed = swerve.targetAngleController.calculate(swerve.getHeading().getRadians(),
            swerve.targetAngle.getRadians());
      }
      swerve.drive(
          new Translation2d(xSpeed, ySpeed),
          angleSpeed,
          driveMode.getAsBoolean());
    } else if (swerve.getAlignRequestActive()) {
      double angleSpeed = swerve.driverTargetAngleController.calculate(swerve.getHeading().getRadians(),
          swerve.targetAngle.getRadians());
      swerve.drive(
          new Translation2d(-swerve.getAlignRequest().getX() * 6, -swerve.getAlignRequest().getY() * 2),
          angleSpeed,
          driveMode.getAsBoolean());
    } else {
      ChassisSpeeds pieceVisionTranslation = new ChassisSpeeds();
      if (PieceVision.getInstance().getYaw() != 0) {
        pieceVisionTranslation = ChassisSpeeds.fromRobotRelativeSpeeds(0, 1, 0, swerve.getPose().getRotation());
      }

      Pose2d realPose2d = swerve.getShooterPose();
      double aimingAngleAdjustmentRadians = 0;
      if (!ShooterTrack.getInstance().getSensor()) {
        aimingAngleAdjustmentRadians = ReverseKinematics.calcRobotAngle(
            ReverseKinematics.convert2dCoords(realPose2d),
            ReverseKinematics.convertSpeed(
                ReverseKinematics.convert2dCoords(realPose2d),
                new ChassisSpeeds()),
            ShooterSpeeds.DRIVE_BY.flywheels) - realPose2d.getRotation().getRadians();

        if (!ShooterPivot.getInstance().notSoFastEggman) {
          ReverseKinematics.calcSubwooferLaunchAngle(realPose2d, new ChassisSpeeds(),
              ShooterSpeeds.DRIVE_BY.flywheels);
        }
      }
      swerve.drive(
          new Translation2d(xSpeed + pieceVisionTranslation.vxMetersPerSecond,
              ySpeed + pieceVisionTranslation.vyMetersPerSecond),
          omega.getAsDouble() * 3 * Math.PI - PieceVision.getInstance().getYaw() / 20
              + aimingAngleAdjustmentRadians / 20,
          driveMode.getAsBoolean());

    }
  }

}
