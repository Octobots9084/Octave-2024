package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Point;
import frc.robot.util.Triangle;

/** Contains tag align data. It contains an area (called hot spot) in which the robot needs to be in
 * order to start aligning it (moving it) to the target position and rotation. */
public enum AlignHotSpots {
        BLUEAMP(new Triangle[] {
                        new Triangle(new Point(0.0, 8.2), new Point(1.84, 4.1), new Point(4.05, 8.2)),
                        new Triangle(new Point(0.0, 0.0), new Point(4.05, 8.2), new Point(4.05, 0)),
        // add more triangles to the hot spot here
        }, new Translation2d(1.84, 8),
                        new Rotation2d(-Math.PI / 2)),
        REDAMP(new Triangle[] {
                        new Triangle(new Point(12.3, 8.2), new Point(14.6, 4.1), new Point(16.5, 8.2)),
                        new Triangle(new Point(16.5, 0), new Point(12.3, 8.2), new Point(12.3, 0)),
        // add more triangles to the hot spot here
        }, new Translation2d(14.6, 8),
                        new Rotation2d(-Math.PI / 2));

        /** The hot spot, represented as a set of triangles on the field. The roboto needs to be inside
         * this area in order to activate the aligning code for the corresponding taret position and
         * rotation.*/
        public Triangle[] hotSpotTriangles;
        /** Target robot position. */
        public Translation2d targetPosition;
        /** Target robot rotation. */
        public Rotation2d targetRotation;

        AlignHotSpots(Triangle[] hotSpotTriangles, Translation2d targetPosition, Rotation2d targetRotation) {
                this.hotSpotTriangles = hotSpotTriangles;
                this.targetPosition = targetPosition;
                this.targetRotation = targetRotation;
        }
}
