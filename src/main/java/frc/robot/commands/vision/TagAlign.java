// package frc.robot.commands.vision;

// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.constants.ArmPositions;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import frc.robot.subsystems.vision.VisionEstimation;

// public class TagAlign extends Command {
//     private VisionEstimation vision;
//     private SwerveSubsystem swerveSubsystem;
//     private PhotonTrackedTarget cameraToTarget;
//     private double ySpeed = 0;

//     public TagAlign() {
//         vision = VisionEstimation.getInstance();
//         swerveSubsystem = SwerveSubsystem.getInstance();
//     }

//     @Override
//     public void initialize() {
//         if (vision.backRightEstimator.hasTargets()) {
//             if (vision.backRightEstimator.getLatestResults().getBestTarget() != null) {
//                 cameraToTarget = vision.backRightEstimator.getLatestResults().getBestTarget();
//             } else {
//                 end(true);
//             }
//         }
//     }

//     @Override
//     public void execute() {
//         try {
//             if (vision.backRightEstimator.hasTargets()) {
//                 var fiducialId = vision.backRightEstimator.getLatestResults().getBestTarget().getFiducialId();
//                 if (vision.backRightEstimator.getLatestResults().getBestTarget() != null
//                         && (fiducialId == 5 || fiducialId == 6)) {
//                     vision.backRightEstimator.getLatestResults().getBestTarget();
//                     ySpeed = (cameraToTarget.getYaw() - 1.5) * 0.1;

//                 } else {
//                     ySpeed = 0;
//                 }

//                 swerveSubsystem.drive(new Translation2d(swerveSubsystem.getRobotVelocity().vxMetersPerSecond, ySpeed),
//                         0, true);
//             }
//         } catch (Exception e) {
//             //
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
//     }
// }
