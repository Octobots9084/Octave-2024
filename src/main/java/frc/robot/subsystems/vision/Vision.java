package frc.robot.subsystems.vision;

import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class Vision implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

  public Vision(PhotonCamera cameraName, Transform3d robotToCamera) {
    this.photonCamera = cameraName;
    PhotonPoseEstimator photonPoseEstimator = null;

    try {
      var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide); // Sets the Origin to the Blue AllianceWall and will be flipped in the robot thread

      photonPoseEstimator = photonCamera != null
          ? new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCamera)
          : null;
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }

    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {
    // Get AprilTag data and updating the pose estimator
    try {
      if (photonPoseEstimator != null && photonCamera != null) {
        var photonResults = photonCamera.getLatestResult(); //Gets the latest camera results

        if (photonResults.hasTargets() && (photonResults.targets.size() > 1
            || photonResults.targets.get(0).getPoseAmbiguity() < VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD)) {
          //Updates the pose estimator
          photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;
            /** 
            * If present then makes sure the measurement is on the field and
            * sets the atomic estimated pose to the current estimated pose
            * from PhotonPoseEstimator
            */
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.LENGTH
                && estimatedPose.getY() > 0.0
                && estimatedPose.getY() <= FieldConstants.WIDTH) {
              atomicEstimatedRobotPose.set(estimatedRobotPose);
            }
          });
        }
      }
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If
   * it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the
   * current alliance is RED.
   * 
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

}
