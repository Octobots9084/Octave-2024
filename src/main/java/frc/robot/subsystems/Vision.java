package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase implements Runnable {
  public static Vision INSTANCE;

  public static Vision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Vision();
    }

    return INSTANCE;
  }

  private final PhotonCamera tagCam = new PhotonCamera("Octocam_2");
  private final PhotonCamera tagCam2 = new PhotonCamera("Octocam_3");
  AprilTagFieldLayout aprilTagFieldLayout = null;
  PhotonPoseEstimator photonPoseEstimatorOne = null;
  PhotonPoseEstimator photonPoseEstimatorTwo = null;

  private List<PhotonTrackedTarget> currentResults;

  public Pose3d currentPoseCamOne = new Pose3d();
  public Pose3d currentPoseCamTwo = new Pose3d();

  public Vision() {
    try {
      Transform3d robotToCamOne = new Transform3d(
          new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(11), Units.inchesToMeters(17)),
          new Rotation3d(0, 0, 0));
      Transform3d robotToCamTwo = new Transform3d(
          new Translation3d(Units.inchesToMeters(-11), Units.inchesToMeters(10), Units.inchesToMeters(17)),
          new Rotation3d(0, 0, 0));
      aprilTagFieldLayout = loadFieldLayout();

      photonPoseEstimatorOne = new PhotonPoseEstimator(aprilTagFieldLayout,
          PoseStrategy.LOWEST_AMBIGUITY, tagCam, robotToCamOne);
      photonPoseEstimatorTwo = new PhotonPoseEstimator(aprilTagFieldLayout,
          PoseStrategy.LOWEST_AMBIGUITY, tagCam2, robotToCamTwo);
    } catch (Exception e) {
      // TODO: handle exception
    }
  }

  @Override
  public void periodic() {
    try {
      if (tagCam.getLatestResult().hasTargets()) {
        currentResults = tagCam.getLatestResult().getTargets();
      }
      if (tagCam2.getLatestResult().hasTargets()) {
        for (int i = 0; i < tagCam2.getLatestResult().getTargets().size() - 1; i++) {
          currentResults.add(tagCam2.getLatestResult().getTargets().get(i));
        }
      }
    } catch (Exception e) {
      // TODO: handle exception
    }
  }

  @Override
  public void run() {
    while (true) {
      try {
        if (!currentResults.isEmpty()) {
          for (int i = 0; i < currentResults.size() - 1; i++) {
            currentResults.remove(
                currentResults.get(i).getPoseAmbiguity() > 0.5 ? currentResults.remove(i) : null);
            i--;
          }
        }
        if (this.getEstimatedGlobalPoseOne(currentPoseCamOne.toPose2d()).isPresent()) {
          currentPoseCamOne = getEstimatedGlobalPoseOne(currentPoseCamOne.toPose2d()).get().estimatedPose;
        }

        if (this.getEstimatedGlobalPoseTwo(currentPoseCamTwo.toPose2d()).isPresent()) {
          currentPoseCamTwo = getEstimatedGlobalPoseTwo(currentPoseCamTwo.toPose2d()).get().estimatedPose;
        }
      } catch (Exception e) {
        // TODO: handle exception
      }
    }
  }

  public Pose3d getCurrentPoseOne() {
    return currentPoseCamOne;
  }

  public Pose3d getCurrentPoseTwo() {
    return currentPoseCamTwo;
  }

  // public PhotonTrackedTarget getBestTarget() {
  //     try {
  //         if (tagCam.getLatestResult().hasTargets()) {
  //             return currentResults.getBestTarget();
  //         }
  //     } catch (Exception e) {
  //         // TODO: handle exception
  //     }

  //     return null;
  // }

  public boolean camHasTarget() {
    if (currentResults == null) {
      return false;
    } else {
      return true;
    }
  }

  private AprilTagFieldLayout loadFieldLayout() {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException ioe) {
      ioe.printStackTrace();
    }
    return new AprilTagFieldLayout(null, 0, 0);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseOne(Pose2d prevEstimatedRobotPose) {
    try {
      photonPoseEstimatorOne.setReferencePose(prevEstimatedRobotPose);
    } catch (Exception e) {
      // TODO: handle exception
    }
    return photonPoseEstimatorOne.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseTwo(Pose2d prevEstimatedRobotPose) {
    try {
      photonPoseEstimatorTwo.setReferencePose(prevEstimatedRobotPose);
    } catch (Exception e) {
      // TODO: handle exception
    }
    return photonPoseEstimatorTwo.update();
  }

}
