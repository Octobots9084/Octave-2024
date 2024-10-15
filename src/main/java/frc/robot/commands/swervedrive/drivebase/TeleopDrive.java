// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ReverseKinematics;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.ReverseKinematics.Box;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.PieceVision;
import frc.robot.subsystems.vision.VisionEstimation;
import frc.robot.util.MathUtil;

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
  private Box blueStageArea = new Box(2.7, 2, 6.5, 5.95);
  private Box redStageArea = new Box(10, 2, 13.9, 5.95);

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
        pieceVisionTranslation = ChassisSpeeds.fromRobotRelativeSpeeds(1.75, 0, 0, swerve.getPose().getRotation());
      }

      Pose2d realPose2d = SwerveSubsystem.getInstance().getShooterPose();
      if (Timer.getFPGATimestamp() > VisionEstimation.getInstance().lastGoodShooterUpdateTime + 0.2) {
        realPose2d = SwerveSubsystem.getInstance().getPose();

      }
      double aimingAngleAdjustmentRadians = 0;

      boolean inShootingArea = false;
      if (Constants.isBlueAlliance) {
        inShootingArea = ReverseKinematics.isPastXAndColliding(realPose2d, 0, 5.5, blueStageArea);
      } else {
        inShootingArea = ReverseKinematics.isPastXAndColliding(realPose2d, 10.7, 16.5, redStageArea);
      }
      if (!ShooterTrack.getInstance().getSensor() && swerve.getTargetSpeaker()) {
        if (inShootingArea) {
          aimingAngleAdjustmentRadians = swerve.targetAngleController.calculate(swerve.getHeading().getRadians(),
              MathUtil.wrapToCircle(ReverseKinematics.calcRobotAngle(
                  ReverseKinematics.convert2dCoords(realPose2d),
                  ReverseKinematics.convertSpeed(
                      ReverseKinematics.convert2dCoords(realPose2d),
                      new ChassisSpeeds()),
                  ShooterSpeeds.DRIVE_BY.flywheels), Math.PI * 2));

          if (!ShooterPivot.getInstance().notSoFastEggman) {
            ShooterPivot.getInstance()
                .setPosition(ReverseKinematics.calcSubwooferLaunchAngle(realPose2d, new ChassisSpeeds(),
                    ShooterSpeeds.DRIVE_BY.flywheels));
          }
        } else {
          ShooterPivot.getInstance().setPosition(ArmPositions.HANDOFF_AND_DEFAULT_SHOT);
        }

      }
      swerve.drive(
          new Translation2d(xSpeed + pieceVisionTranslation.vxMetersPerSecond,
              ySpeed + pieceVisionTranslation.vyMetersPerSecond),
          omega.getAsDouble() * 2 * Math.PI - PieceVision.getInstance().getYaw() / 20
              + aimingAngleAdjustmentRadians / 5,
          driveMode.getAsBoolean());

    }
  }

}
