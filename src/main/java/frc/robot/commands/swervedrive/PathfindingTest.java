package frc.robot.commands.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PathfindingTest {
    // Since we are using a holonomic drivetrain, the rotation component of this pose
// represents the goal holonomic rotation

public static Command getTest() {
    Pose2d targetPose = new Pose2d(1, 1, Rotation2d.fromDegrees(0));

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(

        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    // SmartDashboard.putBoolean("test", true);
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingTest = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        1.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
);
    return pathfindingTest;
}
}
