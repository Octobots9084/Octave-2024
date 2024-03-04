package frc.robot.commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.ArmPositions;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ReverseKinematics;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionEstimation;

public class TagAlign extends Command {
    private VisionEstimation vision;
    private SwerveSubsystem swerveSubsystem;
    ChassisSpeeds realSpeeds;
    Pose2d realPose2d;
    Pose2d target;
    double ySpeed;

    public TagAlign() {
        vision = VisionEstimation.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    // @Override
    // public void initialize() {
    //     if (vision.backRightEstimator.hasTargets()) {
    //         if (vision.backRightEstimator.getLatestResults().getBestTarget() != null) {
    //             cameraToTarget = vision.backRightEstimator.getLatestResults().getBestTarget();
    //         } else {
    //             end(true);
    //         }
    //     }
    // }

    @Override
    public void execute() {
        try {
            realPose2d = swerveSubsystem.getPose();
            realSpeeds = swerveSubsystem.getFieldVelocity();
            target = vision.convert2dCoords(realPose2d);
            ySpeed = -(target.getY() * 3);

            SmartDashboard.putNumber("tagAlignTargetX", target.getX());
            SmartDashboard.putNumber("tagAlignTargetY", target.getY());
            SmartDashboard.putNumber("ySpeed", ySpeed);
            //Replace to not use drive
            swerveSubsystem.drive(new Translation2d(realSpeeds.vxMetersPerSecond, ySpeed), 0,
                    true);
        } catch (Exception e) {
            //
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
