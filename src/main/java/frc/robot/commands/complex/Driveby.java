package frc.robot.commands.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.MathUtil;

public class Driveby extends Command {
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

    public Driveby() {
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
        realPose2d = new Pose2d(4.27, 5.5, swerveSubsystem.getHeading());
        targetPivot = ReverseKinematics.calcSubwooferLaunchAngle(realPose2d, realSpeeds);
        targetFlywheel = ShooterSpeeds.DRIVE_BY.flywheels;
        targetTurn = new Rotation2d(ReverseKinematics.calcRobotAngle(realPose2d, realSpeeds));
    }

    @Override
    public void execute() {
        realSpeeds = swerveSubsystem.getRobotVelocity();
        realPose2d = swerveSubsystem.getPose();
        realPose2d = new Pose2d(4.27, 5.5, swerveSubsystem.getHeading());
        realFlywheel = flywheel.getFlywheelSpeed();
        realPivot = pivot.getPosition();
        ReverseKinematics.configHeightDif(ReverseKinematics.getHeightDif()
                + MathUtil.fitDeadband(ControlMap.CO_DRIVER_RIGHT.getY(), Constants.Climb.ManualDeadband) * 0.05);
        updateTargets();
        pivot.setPosition(targetPivot);

        flywheel.setFlywheelSpeed(targetFlywheel);
        swerveSubsystem.setShootingRequest(targetTurn);
    }

    @Override
    public boolean isFinished() {
        double realRotation = realPose2d.getRotation().getRadians();
        SmartDashboard.putNumber("realFlywheel", realFlywheel);
        SmartDashboard.putNumber("targetFlywheel", targetFlywheel);
        SmartDashboard.putNumber("targetPivot", targetPivot);
        SmartDashboard.putNumber("realPivot", realPivot);
        SmartDashboard.putNumber("realRotation", realPivot);
        SmartDashboard.putNumber("targetRotation", targetTurn.getRadians());

        // turn vs pose2d getturn, flywheelreal vs targetflywheel, pivot vs pivot
        if (MathUtil.isWithinTolerance(realFlywheel, targetFlywheel, 0.01)
                && MathUtil.isWithinTolerance(realPivot, targetPivot, 0.01)

                && MathUtil.isWithinTolerance(realRotation,
                        targetTurn.getRadians(), 0.1)
                && !ShooterTrack.getInstance().getSensor()) {
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
