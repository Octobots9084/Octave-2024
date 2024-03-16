package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.AlignHotSpots;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Point;

public class AmpAlign extends Command {
    private SwerveSubsystem swerveSubsystem;

    public AmpAlign() {
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    @Override
    public void initialize() {
    }

    private boolean isWithinHotSpotArea(AlignHotSpots hotSpots, Pose2d realPose2d) {
        final Point realPose2dAsPoint = new Point(realPose2d.getX(),
                realPose2d.getY());

        for (int i = 0; i < hotSpots.hotSpotTriangles.length; i++) {
            if (hotSpots.hotSpotTriangles[i].isPointInside(realPose2dAsPoint)) {
                return true;
            }
        }
        return false;
    }

    private Pose2d getTargetPose(AlignHotSpots hotSpots, Pose2d realPose2d) {
        return new Pose2d(
                (realPose2d.getX() - hotSpots.targetPosition.getX()) * -3,
                (realPose2d.getY() - hotSpots.targetPosition.getY()) * -3,
                hotSpots.targetRotation);
    }

    @Override
    public void execute() {
        final Pose2d realPose2d = swerveSubsystem.getPose();
        final AlignHotSpots hotSpots = Constants.isBlueAlliance ? AlignHotSpots.BLUEAMP : AlignHotSpots.REDAMP;

        if (isWithinHotSpotArea(hotSpots, realPose2d)) {
            final Pose2d target = getTargetPose(hotSpots, realPose2d);

            // Activate "align request" only when we have a target to set.
            if (!swerveSubsystem.getAlignRequestActive()) {
                swerveSubsystem.setAlignRequestActive(true);
            }
            swerveSubsystem.setAlignRequest(target);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setAlignRequestActive(false);
    }
}
