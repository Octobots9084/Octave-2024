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

            if (Constants.isBlueAlliance) {
                // if (((realPose2d.getX() > 3.06 && realPose2d.getX() < 6.03)
                //         && (realPose2d.getY() > 2.00 && realPose2d.getY() < 6.00))) {
                //     target = new Pose2d((realPose2d.getX() - 4.015) * 5, realPose2d.getY(),
                //             new Rotation2d(
                //                     (Math.atan2((realPose2d.getY() - 4.106), (realPose2d.getX() - 4.065)) - Math.PI)));
                // } else {
                target = new Pose2d((realPose2d.getX() - 1.84) * -5, realPose2d.getY(),
                        new Rotation2d((Math.atan2(realPose2d.getY(), (realPose2d.getX() - 1.84)) - Math.PI)));
                // }

            } else {
                target = new Pose2d((realPose2d.getX() - 14.698) * -5, realPose2d.getY(),
                        new Rotation2d((Math.atan2(realPose2d.getY(), realPose2d.getX()) - Math.PI)));
            }

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
