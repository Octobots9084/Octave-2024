package frc.robot.commands.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.arm.ShooterElevatorPosInstant;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.robot.ControlMap;
import frc.robot.subsystems.ReverseKinematics;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.MathUtil;

public class DrivebyAuto extends Command {
    ShooterPivot pivot;
    ShooterFlywheel flywheel;
    SwerveSubsystem swerveSubsystem;
    ChassisSpeeds realSpeeds;
    Pose2d realPose2d;
    double realPivot;
    double targetPivot;
    double realFlywheel;
    double targetFlywheel;
    Rotation2d targetTurn;

    public DrivebyAuto() {
        pivot = ShooterPivot.getInstance();
        flywheel = ShooterFlywheel.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        swerveSubsystem.setShootingRequestActive(true);
        CommandScheduler.getInstance().schedule(new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT));
        SmartDashboard.putNumber("targetPoseX", 3.0);
    }

    public void updateTargets() {
        realPose2d = SwerveSubsystem.getInstance().getPose();
        realSpeeds = SwerveSubsystem.getInstance().getFieldVelocity();
        targetPivot = ReverseKinematics.calcSubwooferLaunchAngle(realPose2d, realSpeeds,
                ShooterSpeeds.DRIVE_BY.flywheels);
        targetFlywheel = ShooterSpeeds.DRIVE_BY.flywheels;
        targetTurn = new Rotation2d(
                ReverseKinematics.calcRobotAngle(
                        ReverseKinematics.convert2dCoords(swerveSubsystem.getPose()),
                        ReverseKinematics.convertSpeed(ReverseKinematics.convert2dCoords(swerveSubsystem.getPose()),
                                swerveSubsystem.getRobotVelocity()),
                        ShooterSpeeds.DRIVE_BY.flywheels));
        SmartDashboard.putString("realPose2dAhh", realPose2d.toString());
    }

    @Override
    public void execute() {
        realSpeeds = swerveSubsystem.getRobotVelocity();
        realPose2d = swerveSubsystem.getPose();

        realFlywheel = flywheel.getFlywheelSpeedMeters();
        realPivot = pivot.getPosition();
        ReverseKinematics.configHeightDif(ReverseKinematics.getHeightDif()
                + MathUtil.fitDeadband(ControlMap.CO_DRIVER_RIGHT.getY(), Constants.Climb.MANUAL_DEADBAND) * 0.05);
        updateTargets();
        SmartDashboard.putString("realPose2d", realPose2d.toString());

        pivot.setPosition(targetPivot);
        flywheel.setFlyWheelSpeedMeters(targetFlywheel);
        swerveSubsystem.drive(new Translation2d(), swerveSubsystem.targetAngleController.calculate(swerveSubsystem.getHeading().getRadians(), targetTurn.getRadians()), true);
    }

    @Override
    public boolean isFinished() {
        double realRotation = swerveSubsystem.getHeading().getRadians();
        SmartDashboard.putNumber("realFlywheel", realFlywheel);
        SmartDashboard.putNumber("targetFlywheel", targetFlywheel);
        SmartDashboard.putNumber("targetPivot", targetPivot);
        SmartDashboard.putNumber("realPivot", realPivot);
        SmartDashboard.putNumber("realRotation", MathUtil.wrapToCircle(realRotation, 2 * Math.PI));
        SmartDashboard.putNumber("targetRotation", MathUtil.wrapToCircle(targetTurn.getRadians(), 2 * Math.PI));

        // turn vs pose2d getturn, flywheelreal vs targetflywheel, pivot vs pivot
        if (MathUtil.isWithinTolerance(realFlywheel, targetFlywheel, 0.5)
                && MathUtil.isWithinTolerance(realPivot, targetPivot, 0.02)

                && MathUtil.isWithinTolerance(MathUtil.wrapToCircle(realRotation, 2 * Math.PI),
                        MathUtil.wrapToCircle(targetTurn.getRadians(), 2 * Math.PI), 0.01)) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean inturupted) {
        if (!inturupted) {
            CommandScheduler.getInstance().schedule(new TheBigYeet());
        }
        swerveSubsystem.setShootingRequestActive(false);
    }
}
