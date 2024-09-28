package frc.robot.subsystems.vision;

import java.util.concurrent.atomic.AtomicReference;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.MathUtil;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class Vision implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
  private final AtomicReference<EstimatedRobotPose> atomicShooterEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

  public final String cameraName;
  private PhotonPoseEstimator photonPoseEstimatorDriver;

  // Telemetry
  // private final CountPerPeriodTelemetry runCountTelemetry;
  // private final CountPerPeriodTelemetry setAtomicCountTelemetry;

  public Vision(PhotonCamera photonCamera, Transform3d robotToCamera) {
    this.photonCamera = photonCamera;
    cameraName = photonCamera.getName();
    PhotonPoseEstimator photonPoseEstimator = null;
    PhotonPoseEstimator photonPoseEstimatorDriver = null;

    try {
      var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide); // Sets the Origin to the Blue AllianceWall and will
                                                                   // be flipped in the robot thread

      photonPoseEstimator = photonCamera != null
          ? new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCamera)
          : null;
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      photonPoseEstimatorDriver = photonCamera != null
          ? new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCamera)
          : null;
      photonPoseEstimatorDriver.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }

    this.photonPoseEstimator = photonPoseEstimator;
    this.photonPoseEstimatorDriver = photonPoseEstimatorDriver;

    // Initialize telemetry
    // runCountTelemetry = new
    // CountPerPeriodTelemetry(TelemUtils.getCamSDKey(cameraName, "runs per s"), 1);
    // setAtomicCountTelemetry = new
    // CountPerPeriodTelemetry(TelemUtils.getCamSDKey(cameraName, "meas per s
    // push"), 1);
  }

  @Override
  public void run() {
    // Update "run count" telemetry
    // runCountTelemetry.incCount(1);

    // Get AprilTag data and updating the pose estimator
    try {
      if (photonCamera != null) {

        var photonResults = photonCamera.getLatestResult();

        var photonResultsDriver = photonCamera.getLatestResult();
        if (photonPoseEstimator != null) {
          if (photonResults.hasTargets()) {
            for (int i = 0; i < photonResults.targets.size(); i++) {
              if (photonResults.targets.get(i).getPoseAmbiguity() > 0.5
                  || photonResults.targets.get(i).getFiducialId() == 9
                  || photonResults.targets.get(i).getFiducialId() == 15) {
                photonResults.targets.remove(i);
                i++;
              }
            }

            // Updates the pose estimator
            photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
              var estimatedPose = estimatedRobotPose.estimatedPose;

              /**
               * If present then makes sure the measurement is on the field and
               * sets the atomic estimated pose to the current estimated pose
               * from PhotonPoseEstimator
               */
              if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.LENGTH
                  && estimatedPose.getY() > 0.0
                  && estimatedPose.getY() <= FieldConstants.WIDTH
                  && MathUtil.isWithinTolerance(estimatedPose.getZ(), 0, 0.1)) {
                atomicEstimatedRobotPose.set(estimatedRobotPose);

                // Update "set atomic count" telemetry
                // setAtomicCountTelemetry.incCount(1);
              }

            });
          }
        }
        if (photonPoseEstimatorDriver != null) {
          if (photonResultsDriver.hasTargets()) {

            for (int i = 0; i < photonResultsDriver.targets.size(); i++) {
              if (!(photonResultsDriver.targets.get(i).getFiducialId() == 7
                  || photonResultsDriver.targets.get(i).getFiducialId() == 8
                  || photonResultsDriver.targets.get(i).getFiducialId() == 5
                  || photonResultsDriver.targets.get(i).getFiducialId() == 4)) {
                photonResultsDriver.targets.remove(i);

                i++;
              }
            }
            // Updates the pose estimator
            photonPoseEstimatorDriver.update(photonResultsDriver).ifPresent(estimatedRobotPose -> {
              var estimatedPose = estimatedRobotPose.estimatedPose;

              if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.LENGTH
                  && estimatedPose.getY() > 0.0
                  && estimatedPose.getY() <= FieldConstants.WIDTH
                  && MathUtil.isWithinTolerance(estimatedPose.getZ(), 0, 0.1)) {

                atomicShooterEstimatedRobotPose.set(estimatedRobotPose);

                // Update "set atomic count" telemetry
                // setAtomicCountTelemetry.incCount(1);
              }
            });

          }
        }
      }
    } catch (

    Exception e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }

    // Run telemetry
    // runCountTelemetry.periodic();
    // setAtomicCountTelemetry.periodic();
  }

  public boolean hasTargets() {
    return photonCamera.getLatestResult().hasTargets();
  }

  public PhotonPipelineResult getLatestResults() {
    return photonCamera.getLatestResult();
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

  public EstimatedRobotPose grabLatestShooterEstimatedPose() {
    return atomicShooterEstimatedRobotPose.getAndSet(null);
  }
}
