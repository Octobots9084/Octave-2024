package frc.robot.subsystems.vision;

import java.util.concurrent.ExecutionException;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PieceVision extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("PieceCam");

    public static PieceVision INSTANCE;
    private double yaw;

    public static PieceVision getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PieceVision();
            // SmartDashboard.putBoolean("Vision Instance Created", true);
        }

        return INSTANCE;
    }

    public double getYaw() {
        return yaw;
    }

    @Override
    public void periodic() {
        try {
            // Get the latest result from the camera
            var result = camera.getLatestResult();

            // Check if the camera has detected any targets
            if (result.hasTargets()) {
                // Get the best target (closest to the center or largest target)
                PhotonTrackedTarget target = result.getBestTarget();

                // Extract the yaw value from the target
                yaw = target.getYaw();

                // Print or send the yaw value to the SmartDashboard for testing
                SmartDashboard.putNumber("Target Yaw", yaw);

                // Additional processing with yaw can go here
            } else {
                // If no target is detected, set yaw to some default or handle no target
                // detected
                yaw = 0;
                SmartDashboard.putString("Target Status", "No target detected");
            }
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());

        }

    }
}
