package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ReverseKinematics;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TagAlign extends Command {
    private SwerveSubsystem swerveSubsystem;
    ChassisSpeeds realSpeeds;
    Pose2d realPose2d;
    Pose2d target;

    public TagAlign() {
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        swerveSubsystem.setAlignRequestActive(true);
    }

    @Override
    public void execute() {
        try {
            realPose2d = swerveSubsystem.getPose();
            // realSpeeds = swerveSubsystem.getFieldVelocity();

            if (Constants.isBlueAlliance) {
                target = new Pose2d((realPose2d.getX() - 1.84) * -5, realPose2d.getY(),
                        new Rotation2d((Math.atan2(realPose2d.getX(), realPose2d.getY()) - Math.PI)));
            } else {
                target = new Pose2d((realPose2d.getX() - 14.698) * -5, realPose2d.getY(),
                        new Rotation2d((Math.atan2(realPose2d.getX(), realPose2d.getY()) - Math.PI)));
            }

            // targetTurn = new Rotation2d(
            //     ReverseKinematics.calcRobotAngle(
            //             ReverseKinematics.convert2dCoords(swerveSubsystem.getPose()),
            //             ReverseKinematics.convertSpeed(ReverseKinematics.convert2dCoords(swerveSubsystem.getPose()),
            //                     swerveSubsystem.getRobotVelocity()),
            //             ShooterSpeeds.DRIVE_BY.flywheels));

            swerveSubsystem.setAlignRequest(target);
        } catch (Exception e) {
            //
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setAlignRequestActive(false);
    }
}
