package frc.robot.subsystems.vision;

// import static ROBOT_TO_BLINKY

import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.telemetry.CountPerPeriodTelemetry;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class Vision implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

  // Telemetry
  private final CountPerPeriodTelemetry runCountTelemetry;
  private final CountPerPeriodTelemetry setAtomicCountTelemetry;

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

    // Initialize telemetry
    runCountTelemetry = new CountPerPeriodTelemetry("Vision - " + cameraName.getName() + " - runs per s", 1);
    setAtomicCountTelemetry = new CountPerPeriodTelemetry(
        "Vision - " + cameraName.getName() + " - set atomic count per s",
        1);
  }

  @Override
  public void run() {
    // Update "run count" telemetry
    runCountTelemetry.incCount(1);

    // Get AprilTag data and updating the pose estimator
    try {
      if (photonPoseEstimator != null && photonCamera != null) {
        var photonResults = photonCamera.getLatestResult(); //Gets the latest camera results

        // var test = VisionConstants.class.getDeclaredField("ROBOT_TO_" + photonCamera.getName().toUpperCase());
        // test.setAccessible(true);
        // Transform3d test2 = (Transform3d) test.get(VisionConstants.class.getClass());
        //SmartDashboard.putString("Test", test2.toString());
        SwerveSubsystem.getInstance().getSwerveDrive().field.getObject("vision/" + photonCamera.getName()).setPose(
            photonResults.getBestTarget().getBestCameraToTarget().getX(),
            photonResults.getBestTarget().getBestCameraToTarget().getY(),
            photonResults.getBestTarget().getBestCameraToTarget().getRotation().toRotation2d());

        if (photonResults
            .hasTargets()) {
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

              // Update "set atomic count" telemetry
              setAtomicCountTelemetry.incCount(1);
            }
          });
        }
      }
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }

    // Run telemetry
    runCountTelemetry.periodic();
    setAtomicCountTelemetry.periodic();
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
