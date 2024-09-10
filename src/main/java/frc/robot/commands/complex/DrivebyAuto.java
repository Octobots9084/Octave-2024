package frc.robot.commands.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class DrivebyAuto extends Command {
    private int count = 0;
    private static int count2 = 0;
    private static final double maintainToleranceTime = 0.1;

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
    private double initialToleranceTime = 0;

    public DrivebyAuto(boolean first) {
        pivot = ShooterPivot.getInstance();
        flywheel = ShooterFlywheel.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();
        this.first = first;
        count++;
        // SmartDashboard.putNumber("Driveby", SmartDashboard.getNumber("Driveby", 0) +
        // 1);
        System.out.println("drivebyconst");
        System.out.println("driveinit " + count + " " + count2);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new ShooterElevatorPosInstant(ArmPositions.HANDOFF_AND_DEFAULT_SHOT));
        // SmartDashboard.putNumber("targetPoseX", 3.0);
        Light.getInstance().setAnimation(Animations.AIMING);
        count2++;
        System.out.println("driveinit " + count + " " + count2);

    }

    public void updateTargets() {
        realPose2d = SwerveSubsystem.getInstance().getPose();
        realSpeeds = SwerveSubsystem.getInstance().getFieldVelocity();
        if (first) {
            targetPivot = ReverseKinematics.calcSubwooferLaunchAngle(realPose2d, realSpeeds,
                    ShooterSpeeds.DRIVE_BY.flywheels - 20);
            targetFlywheel = ShooterSpeeds.DRIVE_BY.flywheels - 20;

        } else {
            targetPivot = ReverseKinematics.calcSubwooferLaunchAngle(realPose2d, realSpeeds,
                    ShooterSpeeds.DRIVE_BY.flywheels);
            targetFlywheel = ShooterSpeeds.DRIVE_BY.flywheels;

        }
        targetTurn = new Rotation2d(
                ReverseKinematics.calcRobotAngle(
                        ReverseKinematics.convert2dCoords(swerveSubsystem.getShooterPose()),
                        ReverseKinematics.convertSpeed(
                                ReverseKinematics.convert2dCoords(swerveSubsystem.getShooterPose()),
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
        // System.out.println("driveexe " + count + " " + count2);
    }

    @Override
    public boolean isFinished() {
        double realRotation = swerveSubsystem.getHeading().getRadians();
        // SmartDashboard.putNumber("targetFlywheel", targetFlywheel);

        // SmartDashboard.putNumber("targetPivot", targetPivot);
        // SmartDashboard.putNumber("realPivot", realPivot);
        // SmartDashboard.putNumber("realRotation", MathUtil.wrapToCircle(realRotation,
        // 2 * Math.PI));
        // SmartDashboard.putNumber("targetRotation",
        // MathUtil.wrapToCircle(targetTurn.getRadians(), 2 * Math.PI));
        // SmartDashboard.putNumber("realFlywheel", realFlywheel);
        // SmartDashboard.putNumber("targetFlywheel", targetFlywheel);

        // SmartDashboard.putBoolean("flywheelTolerance",
        // MathUtil.isWithinTolerance(realFlywheel, targetFlywheel, 2));
        // SmartDashboard.putBoolean("pivotTolerance",
        // MathUtil.isWithinTolerance(realPivot, targetPivot, 0.005));
        // SmartDashboard.putBoolean("rotationTolerance", isInTolerance(realRotation));

        // turn vs pose2d getturn, flywheelreal vs targetflywheel, pivot vs pivot
        if (longTolerance(realRotation)) {
            Light.getInstance().setAnimation(Animations.SHOT_READY);
            System.out.println("Teleop shot authorized. Flywheels at " + targetFlywheel + " of " + realFlywheel
                    + " with a tolerance of " + flywheelTolerance + " and error of " + (targetFlywheel - realFlywheel)
                    + " Pivot at " + realPivot + " of " + targetPivot + " with a tolerance of " + pivotTolerance
                    + " and an error of " + (targetPivot - realPivot) + " Bot rotation at "
                    + realPose2d.getRotation().getRadians() + " of " + targetTurn.getRadians() + " with a tolerance of "
                    + rotationTolerance + " and an error of "
                    + (realPose2d.getRotation().getRadians() - targetTurn.getRadians()) + ". Good luck!");

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

    private double flywheelTolerance = 0.2;
    private double pivotTolerance = 0.005;
    private double rotationTolerance = 0.05;

    private boolean longTolerance(double realFlywheel) {
        if (isInTolerance(realFlywheel)) {
            if (initialToleranceTime == 0) {
                initialToleranceTime = Timer.getFPGATimestamp();
            } else if (Timer.getFPGATimestamp() - initialToleranceTime > maintainToleranceTime) {
                System.out.println("Autonomous shot authorized. Flywheels at " + targetFlywheel + " of " + realFlywheel
                        + " with a tolerance of " + flywheelTolerance + " Pivot at " + realPivot + " of " + targetPivot
                        + " with a tolerance of " + pivotTolerance + " Bot rotation at "
                        + realPose2d.getRotation().getRadians() + " of " + targetTurn.getRadians()
                        + " with a tolerance of " + rotationTolerance + ". Good luck!");
                return true;
            }
        } else {
            initialToleranceTime = 0;
        }

        return false;
    }

    @Override
    public void end(boolean inturupted) {
        CommandScheduler.getInstance().schedule(new TheBigYeetAuto());
        swerveSubsystem.setShootingRequestActive(false);
        swerveSubsystem.targetAngleEnabled = false;
        System.out.println("Auto shot canceled. Flywheels at " + targetFlywheel + " of " + realFlywheel
                + " with a tolerance of " + flywheelTolerance + " and error of " + (targetFlywheel - realFlywheel)
                + " Pivot at " + realPivot + " of " + targetPivot + " with a tolerance of " + pivotTolerance
                + " and an error of " + (targetPivot - realPivot) + " Bot rotation at "
                + realPose2d.getRotation().getRadians() + " of " + targetTurn.getRadians() + " with a tolerance of "
                + rotationTolerance + " and an error of "
                + (realPose2d.getRotation().getRadians() - targetTurn.getRadians()) + ". RIP");

        System.out.println("drivebyend " + inturupted + count + " " + count2);
    }
}
