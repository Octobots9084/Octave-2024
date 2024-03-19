package frc.robot.commands.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.MathUtil;

public class FerryShot extends Command {
    private static final double maintainToleranceTime = 0.1;
    ShooterPivot pivot;
    ShooterFlywheel flywheel;
    SwerveSubsystem swerveSubsystem;
    ChassisSpeeds realSpeeds;
    Pose2d realPose2d;
    double realPivot;
    double targetPivot;
    double realFlywheel;
    Pose2d targetLanding;
    double targetFlywheel;
    Rotation2d targetTurn;
    private double initialToleranceTime = 0;

    public FerryShot() {
        pivot = ShooterPivot.getInstance();
        flywheel = ShooterFlywheel.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        swerveSubsystem.setShootingRequestActive(true);
        CommandScheduler.getInstance().schedule(new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT));
        // SmartDashboard.putNumber("targetPoseX", 3.0);
        Light.getInstance().setAnimation(Animations.AIMING);
    }

    public void updateTargets() {
        realPose2d = SwerveSubsystem.getInstance().getPose();
        realSpeeds = SwerveSubsystem.getInstance().getFieldVelocity();
        realFlywheel = flywheel.getFlywheelSpeed();
        targetPivot = ReverseKinematics.calcFerryLaunchAngle(realPose2d, targetLanding, realFlywheel);
        targetFlywheel = ShooterSpeeds.FERRY_SHOT.flywheels;
        targetTurn = new Rotation2d(
                ReverseKinematics.calcFerryRotation(realPose2d, targetLanding)
        );
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
        // SmartDashboard.putString("realPose2d", realPose2d.toString());
        swerveSubsystem.setShootingRequest(targetTurn);
        flywheel.setFlyWheelSpeedMeters(targetFlywheel);
        if (!pivot.notSoFastEggman) {
            pivot.setPosition(targetPivot);
        }
    }

    @Override
    public boolean isFinished() {
        double realRotation = swerveSubsystem.getHeading().getRadians();
        // SmartDashboard.putNumber("targetFlywheel", targetFlywheel);

        SmartDashboard.putNumber("targetPivot", targetPivot);
        SmartDashboard.putNumber("realPivot", realPivot);
        SmartDashboard.putNumber("realRotation", MathUtil.wrapToCircle(realRotation, 2 * Math.PI));
        SmartDashboard.putNumber("targetRotation", MathUtil.wrapToCircle(targetTurn.getRadians(), 2 * Math.PI));
        SmartDashboard.putNumber("realFlywheel", realFlywheel);
        SmartDashboard.putNumber("targetFlywheel", targetFlywheel);

        SmartDashboard.putNumber("flywheelTolerance", MathUtil.isWithinTolerance(realFlywheel, targetFlywheel, 2) ? 1 : 0);
        SmartDashboard.putNumber("pivotTolerance", MathUtil.isWithinTolerance(realPivot, targetPivot, 0.005) ? 1 : 0);
        SmartDashboard.putNumber("rotationTolerance", isInTolerance(realRotation) ? 1 : 0);

        // turn vs pose2d getturn, flywheelreal vs targetflywheel, pivot vs pivot
        if (longTolerance(realRotation)) {
            Light.getInstance().setAnimation(Animations.SHOT_READY);
            return true;
        } else {
            return false;
        }
    }

    private boolean isInTolerance(double realRotation) {
        return (MathUtil.isWithinTolerance(realFlywheel, targetFlywheel, 2)
                && MathUtil.isWithinTolerance(realPivot, targetPivot, 0.005)

                && MathUtil.isWithinTolerance(MathUtil.wrapToCircle(realRotation, 2 * Math.PI),
                        MathUtil.wrapToCircle(targetTurn.getRadians(), 2 * Math.PI), 0.05));
    }

    private boolean longTolerance(double realFlywheel) {
        if (isInTolerance(realFlywheel)) {
            if (initialToleranceTime == 0) {
                initialToleranceTime = Timer.getFPGATimestamp();
            } else if (Timer.getFPGATimestamp() - initialToleranceTime > maintainToleranceTime) {
                return true;
            }
        } else {
            initialToleranceTime = 0;
        }

        return false;
    }

    @Override
    public void end(boolean inturupted) {
        if (!inturupted) {
            CommandScheduler.getInstance().schedule(new TheBigYeet());
        }
        swerveSubsystem.setShootingRequestActive(false);
        swerveSubsystem.targetAngleEnabled = false;
    }
}
