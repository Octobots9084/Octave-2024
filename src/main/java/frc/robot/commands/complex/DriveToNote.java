package frc.robot.commands.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterSpeeds;
import frc.robot.subsystems.ReverseKinematics;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterTrack;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.PieceVision;
import frc.robot.subsystems.vision.VisionEstimation;
import frc.robot.util.MathUtil;

public class DriveToNote extends Command {
    SwerveSubsystem swerve;
    double randomRotation = 0;
    public DriveToNote() {
        swerve = SwerveSubsystem.getInstance();
    }

    @Override
    public void execute() {
        ChassisSpeeds pieceVisionTranslation = new ChassisSpeeds();
      if (PieceVision.getInstance().getYaw() != 0) {
        pieceVisionTranslation = ChassisSpeeds.fromRobotRelativeSpeeds(2.5, 0, 0, swerve.getPose().getRotation());
      } else {
        randomRotation = Math.random() - 0.5;
      }
        swerve.drive(new Translation2d(pieceVisionTranslation.vxMetersPerSecond,
            pieceVisionTranslation.vyMetersPerSecond),
        - PieceVision.getInstance().getYaw() / 15 + randomRotation,
        true);

    
    }
}
