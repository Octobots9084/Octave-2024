package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum AlignHotSpots {
    // {x1, y1, x2, y2, x3, y3}
    BLUEAMP(new double[] { 0.0, 8.186, 1.84, 4.1, 4.05, 8.186 }, new Translation2d(1.84, Double.MAX_VALUE),
            new Rotation2d(-Math.PI / 2));

    public double[] triangle;
    public Translation2d targetPosition;
    public Rotation2d targetRotation;

    AlignHotSpots(double[] hotSpotTriangle, Translation2d targetPosition, Rotation2d targetRotation) {
        this.triangle = hotSpotTriangle;
        this.targetPosition = targetPosition;
        this.targetRotation = targetRotation;
    }
}
