package frc.robot.constants;

import java.lang.reflect.Field;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.Point;
import frc.robot.util.Triangle;

/** Contains tag align data. It contains an area (called hot spot) in which the robot needs to be in
 * order to start aligning it (moving it) to the target position and rotation. 
 * 
 * Link: https://www.desmos.com/calculator/p8htevfsmd
 * 
 * */

public class AlignHotSpots {
    public static final AlignHotSpots BLUEAMP = new AlignHotSpots(new Triangle[] {
            new Triangle(new Point(0.0, 8.2), new Point(1.84, 4.1), new Point(4.05, 8.2)),
            new Triangle(new Point(0.0, 0.0), new Point(4.05, 8.2), new Point(4.05, 0)),
            // add more triangles to the hot spot here
    }, new Translation2d(1.88, 8),
            new Rotation2d(-Math.PI / 2));

    public static final AlignHotSpots BlueClimbOne = new AlignHotSpots(
            new Triangle[] { new Triangle(new Point(3.17, 4.08), new Point(0, 4.14), new Point(0, 0)),
                    new Triangle(new Point(0, 0), new Point(3.17, 4.08), new Point(5.86, 2.46)),
                    new Triangle(new Point(0, 0), new Point(8.23, 0), new Point(5.86, 2.46)),
            },
            new Translation2d(4.5, 3.5), new Rotation2d(-2 * Math.PI / 3));

    public static final AlignHotSpots BlueClimbTwo = new AlignHotSpots(
            new Triangle[] { new Triangle(new Point(3.17, 4.08), new Point(0, 4.14), new Point(0, 8.2)),
                    new Triangle(new Point(0, 8.2), new Point(3.17, 4.08), new Point(5.86, 5.78)),
                    new Triangle(new Point(0, 8.2), new Point(8.23, 8.2), new Point(5.86, 5.78)),
            },
            new Translation2d(4.5, FieldConstants.WIDTH - 3.5), new Rotation2d(2 * Math.PI / 3));

    public static final AlignHotSpots BlueClimbThree = new AlignHotSpots(
            new Triangle[] { new Triangle(new Point(5.86, 2.46), new Point(8.23, 8.2),
                    new Point(5.86, 5.78)),
                    new Triangle(new Point(5.86, 2.46), new Point(8.23, 8.2), new Point(8.23, 0)),
                    new Triangle(new Point(10.6, 0), new Point(8.23, 8.2), new Point(8.23, 0)),
                    new Triangle(new Point(10.6, 0), new Point(8.23, 8.2), new Point(10.6, 8.2)),
            },
            new Translation2d(5.55, FieldConstants.WIDTH / 2), new Rotation2d(0));

    // REDAMP(new Triangle[]{new Triangle(new Point(12.3,8.2),new Point(14.6,4.1),new Point(16.5,8.2)),new Triangle(new Point(16.5,0),new Point(12.3,8.2),new Point(12.3,0)),
    // // add more triangles to the hot spot here
    // },new Translation2d(14.6,8),new Rotation2d(-Math.PI/2));

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
