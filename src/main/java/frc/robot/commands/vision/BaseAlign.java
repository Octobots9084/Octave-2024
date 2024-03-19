package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlignHotSpots;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.MathUtil;
import frc.robot.util.Point;

public abstract class BaseAlign extends Command {
    protected SwerveSubsystem swerveSubsystem;

    public BaseAlign() {
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    protected boolean isWithinHotSpotArea(AlignHotSpots hotSpots, Pose2d realPose2d) {
        final Point realPose2dAsPoint = new Point(realPose2d.getX(),
                realPose2d.getY());

        for (int i = 0; i < hotSpots.hotSpotTriangles.length; i++) {
            if (hotSpots.hotSpotTriangles[i].isPointInside(realPose2dAsPoint)) {
                return true;
            }
        }
        return false;
    }

    protected Pose2d getTargetPose(AlignHotSpots hotSpots, Pose2d realPose2d) {
        //Cannot be even powers b/c we want to preserve the sign
        return new Pose2d(
                -MathUtil.powPreserveSign(realPose2d.getX() - hotSpots.targetPosition.getX(), 2),
                -MathUtil.powPreserveSign(realPose2d.getY() - hotSpots.targetPosition.getY(), 2),
                hotSpots.targetRotation);
    }

    protected abstract AlignHotSpots[] getHotSpots();

    @Override
    public void execute() {
        final Pose2d realPose2d = swerveSubsystem.getPose();
        final AlignHotSpots[] allHotSpots = getHotSpots();

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
