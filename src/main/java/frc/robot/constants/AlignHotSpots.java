package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.Point;
import frc.robot.util.Triangle;

/**
 * Contains tag align data. It contains an area (called hot spot) in which the
 * robot needs to be in
 * order to start aligning it (moving it) to the target position and rotation.
 * 
 * Link: https://www.desmos.com/calculator/xwrjvianod
 * 
 */

public class AlignHotSpots {
        public static final AlignHotSpots BlueAmp = new AlignHotSpots(new Triangle[] {
                        new Triangle(new Point(0.0, FieldConstants.WIDTH), new Point(1.84, 4.1),
                                        new Point(4.05, FieldConstants.WIDTH)),
                        new Triangle(new Point(0.0, 0.0), new Point(4.05, FieldConstants.WIDTH), new Point(4.05, 0)),
        }, new Translation2d(2.3, FieldConstants.WIDTH),
                        new Rotation2d(-Math.PI / 2));

        public static final AlignHotSpots BlueClimbOne = new AlignHotSpots(
                        new Triangle[] { new Triangle(new Point(3.17, 4.08), new Point(0, 4.14), new Point(0, 0)),
                                        new Triangle(new Point(0, 0), new Point(3.17, 4.08), new Point(5.86, 2.46)),
                                        new Triangle(new Point(0, 0), new Point(8.23, 0), new Point(5.86, 2.46)),
                        },
                        new Translation2d(4.4, 3.2), new Rotation2d(-2 * Math.PI / 3));

        public static final AlignHotSpots BlueClimbTwo = new AlignHotSpots(
                        new Triangle[] {
                                        new Triangle(new Point(3.17, 4.08), new Point(0, 4.14),
                                                        new Point(0, FieldConstants.WIDTH)),
                                        new Triangle(new Point(0, FieldConstants.WIDTH), new Point(3.17, 4.08),
                                                        new Point(5.86, 5.78)),
                                        new Triangle(new Point(0, FieldConstants.WIDTH),
                                                        new Point(8.23, FieldConstants.WIDTH),
                                                        new Point(5.86, 5.78)),
                        },
                        new Translation2d(4.4, FieldConstants.WIDTH - 3.2), new Rotation2d(2 * Math.PI / 3));

        public static final AlignHotSpots BlueClimbThree = new AlignHotSpots(
                        new Triangle[] { new Triangle(new Point(5.86, 2.46), new Point(8.23, FieldConstants.WIDTH),
                                        new Point(5.86, 5.78)),
                                        new Triangle(new Point(5.86, 2.46), new Point(8.23, FieldConstants.WIDTH),
                                                        new Point(8.23, 0)),
                                        new Triangle(new Point(10.6, 0), new Point(8.23, FieldConstants.WIDTH),
                                                        new Point(8.23, 0)),
                                        new Triangle(new Point(10.6, 0), new Point(8.23, FieldConstants.WIDTH),
                                                        new Point(10.6, FieldConstants.WIDTH)),
                        },
                        new Translation2d(5.85, FieldConstants.WIDTH / 2), new Rotation2d(0));

        public static final AlignHotSpots BlueSource = new AlignHotSpots(new Triangle[] {
                        new Triangle(new Point(FieldConstants.LENGTH, 6.1), new Point(FieldConstants.LENGTH, 0),
                                        new Point(5.86, 0)),
                        new Triangle(new Point(5.86, 0.0), new Point(5.86, 2.68), new Point(10.65, 2.75)),
        }, new Translation2d(15.3, 1.1),
                        new Rotation2d(2 * Math.PI / 3));

        // REDAMP(new Triangle[]{new Triangle(new Point(12.3,8.2),new
        // Point(14.6,4.1),new Point(16.5,8.2)),new Triangle(new Point(16.5,0),new
        // Point(12.3,8.2),new Point(12.3,0)),
        // // add more triangles to the hot spot here
        // },new Translation2d(14.6,8),new Rotation2d(-Math.PI/2));

        /**
         * The hot spot, represented as a set of triangles on the field. The roboto
         * needs to be inside
         * this area in order to activate the aligning code for the corresponding taret
         * position and
         * rotation.
         */
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

        public static AlignHotSpots getRedFromBlue(AlignHotSpots blue) {
                final Triangle[] redTriangles = new Triangle[blue.hotSpotTriangles.length];
                final Translation2d redPosition = new Translation2d(
                                FieldConstants.LENGTH - blue.targetPosition.getX(), blue.targetPosition.getY());
                final Rotation2d redRotation = new Rotation2d(blue.targetRotation.getRadians());

                for (int i = 0; i < redTriangles.length; i++) {
                        redTriangles[i] = new Triangle(new Point(
                                        FieldConstants.LENGTH - blue.hotSpotTriangles[i].point1.x,
                                        blue.hotSpotTriangles[i].point1.y),
                                        new Point(
                                                        FieldConstants.LENGTH - blue.hotSpotTriangles[i].point2.x,
                                                        blue.hotSpotTriangles[i].point2.y),
                                        new Point(
                                                        FieldConstants.LENGTH - blue.hotSpotTriangles[i].point3.x,
                                                        blue.hotSpotTriangles[i].point3.y));
                }

                return new AlignHotSpots(redTriangles, redPosition, redRotation);
        }
}
