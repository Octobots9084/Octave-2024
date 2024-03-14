package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.constants.AlignHotSpots;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.MathUtil;

public class AmpAlign extends Command {
    private SwerveSubsystem swerveSubsystem;
    ChassisSpeeds realSpeeds;
    Pose2d realPose2d;
    Pose2d target;

    // HotSpot {
    //     triangles: Triangle[]
    //     turnToAngle: double
    // }

    // blueAmpHotSpot = {
    //     triangles: [ trianghles, trinagle2]
    //     turnToANgle: 45
    // }

    // blueSourceHotSPot ...

    // blueCLimb1Hotpot
    // blueCLimb2HotSPot

    // private static final HotSpot[] blueTurnTo = [
    //   blueAmptHotSpot,
    //   blueSourceHotSpot,
    //   blueCLimb1Hotpot,
    // ];

    public AmpAlign() {
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        swerveSubsystem.setAlignRequestActive(true);
    }

    @Override
    public void execute() {
        try {
            realPose2d = swerveSubsystem.getPose();

            if (Constants.isBlueAlliance) {

                if (MathUtil.isPointInTriangle(AlignHotSpots.BLUEAMP.triangle[0],
                        AlignHotSpots.BLUEAMP.triangle[1],
                        AlignHotSpots.BLUEAMP.triangle[2],
                        AlignHotSpots.BLUEAMP.triangle[3],
                        AlignHotSpots.BLUEAMP.triangle[4],
                        AlignHotSpots.BLUEAMP.triangle[5],
                        realPose2d.getX(),
                        realPose2d.getY())) {

                    target = new Pose2d((realPose2d.getX() - 1.84) * -5, realPose2d.getY(),
                            new Rotation2d(
                                    (Math.atan2(realPose2d.getY() - 8.22, (realPose2d.getX() - 1.84)) - Math.PI)));
                }

            } else {
                target = new Pose2d((realPose2d.getX() - 14.698) * -5, realPose2d.getY(),
                        new Rotation2d((Math.atan2(realPose2d.getY() - 8.22, realPose2d.getX() - 14.698) - Math.PI)));
            }

            swerveSubsystem.setAlignRequest(target);
        } catch (Exception e) {
            //
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setAlignRequestActive(false);
    }
}
