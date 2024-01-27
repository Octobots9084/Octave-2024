package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TestingPaths extends SequentialCommandGroup{
    
    public TestingPaths() {
        addCommands(new InstantCommand(()->{
            SwerveSubsystem.getInstance().resetOdometry(new Pose2d(.7, 4.37, new Rotation2d(120)));
        }),piece1(), piece2(), piece3());
    }

    public Command piece1() {
        return AutoBuilder.pathfindToPose(new Pose2d(2.7, 3.6, new Rotation2d(0)), new PathConstraints(1, 2, 1, 2),1);
    }

    public Command piece2() {
        return AutoBuilder.pathfindToPose(new Pose2d(9.25, -.5, new Rotation2d(0)), new PathConstraints(1, 2, 1, 2), 1);
    }

    public Command piece3() {
        return AutoBuilder.pathfindToPose(new Pose2d(9.25, 1.5, new Rotation2d(0)), new PathConstraints(1, 2, 1, 2));
    }

}
