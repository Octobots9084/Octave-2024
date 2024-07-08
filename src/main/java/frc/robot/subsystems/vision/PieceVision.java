package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PieceVision extends SubsystemBase {
    private final PhotonCamera pieceCam;
    private PhotonPipelineResult allResults;
    private static PieceVision INSTANCE;

    public static PieceVision getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PieceVision();
        }
        return INSTANCE;
    }

    public PieceVision() {
        this.pieceCam = new PhotonCamera("PieceCam");
    }

    @Override
    public void periodic() {
        allResults = pieceCam.getLatestResult();
    }

    public PhotonCamera getCamera() {
        return this.pieceCam;
    }

    public PhotonPipelineResult getAllResults() {
        return allResults;
    }

    public PhotonTrackedTarget getBestPiece() {
        return allResults.getBestTarget();
    }

}
