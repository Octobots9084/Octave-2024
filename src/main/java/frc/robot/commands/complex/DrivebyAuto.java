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
import frc.robot.subsystems.lights.Animations;
import frc.robot.subsystems.lights.Light;
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
    boolean first;

    public DrivebyAuto(boolean first) {
        pivot = ShooterPivot.getInstance();
        flywheel = ShooterFlywheel.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();
        this.first = first;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT));
        // SmartDashboard.putNumber("targetPoseX", 3.0);
        Light.getInstance().setAnimation(Animations.AIMING);
    }

    public void updateTargets() {
        realPose2d = SwerveSubsystem.getInstance().getPose();
        realSpeeds = SwerveSubsystem.getInstance().getFieldVelocity();
        if(first) {
        targetPivot = ReverseKinematics.calcSubwooferLaunchAngle(realPose2d, realSpeeds,
                ShooterSpeeds.DRIVE_BY.flywheels - 20);
        targetFlywheel = ShooterSpeeds.DRIVE_BY.flywheels - 20;

        }
        else{
        targetPivot = ReverseKinematics.calcSubwooferLaunchAngle(realPose2d, realSpeeds,
                ShooterSpeeds.DRIVE_BY.flywheels);
        targetFlywheel = ShooterSpeeds.DRIVE_BY.flywheels;

        }
        targetTurn = new Rotation2d(
                ReverseKinematics.calcRobotAngle(
                        ReverseKinematics.convert2dCoords(swerveSubsystem.getPose()),
                        ReverseKinematics.convertSpeed(ReverseKinematics.convert2dCoords(swerveSubsystem.getPose()),
                                swerveSubsystem.getRobotVelocity()),
                        targetFlywheel));
        // SmartDashboard.putString("realPose2dAhh", realPose2d.toString());
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
        flywheel.setFlyWheelSpeedMeters(targetFlywheel);
        pivot.setPosition(targetPivot);
        swerveSubsystem.drive(new Translation2d(), swerveSubsystem.targetAngleController
                .calculate(swerveSubsystem.getHeading().getRadians(), targetTurn.getRadians()), true);
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

        SmartDashboard.putBoolean("flywheelTolerance", MathUtil.isWithinTolerance(realFlywheel, targetFlywheel, 2));
        SmartDashboard.putBoolean("pivotTolerance", MathUtil.isWithinTolerance(realPivot, targetPivot, 0.005));
        SmartDashboard.putBoolean("rotationTolerance", isInTolerance(realRotation));

        // turn vs pose2d getturn, flywheelreal vs targetflywheel, pivot vs pivot
        if (isInTolerance(realRotation)) {
            Light.getInstance().setAnimation(Animations.SHOT_READY);
            return true;
        } else {
            return false;
        }
    }

    private boolean isInTolerance(double realRotation) {
        if (first) {
            
        return (MathUtil.isWithinTolerance(realFlywheel, targetFlywheel, 2)
                && MathUtil.isWithinTolerance(realPivot, targetPivot, 0.005));
        }
        return (MathUtil.isWithinTolerance(realFlywheel, targetFlywheel, 2)
                && MathUtil.isWithinTolerance(realPivot, targetPivot, 0.005)

                && MathUtil.isWithinTolerance(MathUtil.wrapToCircle(realRotation, 2 * Math.PI),
                        MathUtil.wrapToCircle(targetTurn.getRadians(), 2 * Math.PI), 0.05));
    }

    @Override
    public void end(boolean inturupted) {
        CommandScheduler.getInstance().schedule(new TheBigYeet());
        swerveSubsystem.setShootingRequestActive(false);
        swerveSubsystem.targetAngleEnabled = false;
    }
}
