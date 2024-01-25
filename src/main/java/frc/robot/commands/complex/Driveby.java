package frc.robot.commands.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.MathUtil;

public class Driveby extends Command{
    ShooterPivot pivot;
    ShooterFlywheel flywheel;
    SwerveSubsystem swerveSubsystem;
    ChassisSpeeds realSpeeds;
    Pose2d realPose2d;
    double realPivot;
    double targetPivot;
    double realFlywheel;
    double targetFlywheel;
    Rotation2d targetTurn;


    public Driveby (){
        pivot = ShooterPivot.getInstance();
        flywheel = ShooterFlywheel.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    public void updateTargets(){
        targetPivot = 0;
        targetFlywheel = 0;
        targetTurn = new Rotation2d();
    }
    
    @Override
    public boolean isFinished(){
        realSpeeds = swerveSubsystem.getRobotVelocity();
        realPose2d = swerveSubsystem.getPose();
        realFlywheel = flywheel.getFlywheelSpeed();
        realPivot = pivot.getPosition();
        updateTargets();
        // turn vs pose2d getturn, flywheelreal vs targetflywheel, pivot vs pivot
        if (MathUtil.isWithinTolerance(realFlywheel, targetFlywheel, 0.1) && MathUtil.isWithinTolerance(realPivot, targetPivot, 0.1) && MathUtil.isWithinTolerance(realPose2d.getRotation().getRadians(), targetTurn.getRadians(), 0.1)) {
            return true;
        }
        else{
            pivot.setPosition(targetPivot);
            flywheel.setFlywheelSpeed(targetFlywheel);
            swerveSubsystem.setShootingRequest(targetTurn);
            return false;
        }
    }
    
    @Override
    public void end (boolean inturupted){
        if (!inturupted){
            CommandScheduler.getInstance().schedule(new TheBigYeet());
        }
    }
}
