package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.AlignHotSpots;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Point;

public class ClimbAlign extends Command {
    private SwerveSubsystem swerveSubsystem;

    public ClimbAlign() {
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
        final AlignHotSpots hotSpotsOne = Constants.isBlueAlliance ? AlignHotSpots.BlueClimbOne
                : AlignHotSpots.getRedFromBlue(AlignHotSpots.BlueClimbOne);
        final AlignHotSpots hotSpotsTwo = Constants.isBlueAlliance ? AlignHotSpots.BlueClimbTwo
                : AlignHotSpots.getRedFromBlue(AlignHotSpots.BlueClimbTwo);
        final AlignHotSpots hotSpotsThree = Constants.isBlueAlliance ? AlignHotSpots.BlueClimbThree
                : AlignHotSpots.getRedFromBlue(AlignHotSpots.BlueClimbThree);

        final AlignHotSpots[] allHotSpots = { hotSpotsOne, hotSpotsTwo, hotSpotsThree };

        for (int i = 0; i < allHotSpots.length; i++) {
            if (isWithinHotSpotArea(allHotSpots[i], realPose2d)) {
                final Pose2d target = getTargetPose(allHotSpots[i], realPose2d);

                // Activate "align request" only when we have a target to set.
                if (!swerveSubsystem.getAlignRequestActive()) {
                    swerveSubsystem.setAlignRequestActive(true);
                }
                swerveSubsystem.setAlignRequest(target);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setAlignRequestActive(false);
    }
}
