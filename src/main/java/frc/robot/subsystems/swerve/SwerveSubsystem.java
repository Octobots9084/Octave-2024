package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{

    //Variables
    private final SwerveDrive swerveDrive;
    // private SwerveAutoBuilder autoBuilder = null;
    Translation2d adjustedTranslation2d = new Translation2d(0,0);

    public double maxSpeed = Units.feetToMeters(2);
    public SwerveSubsystem(File directory){
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try
        {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(maxSpeed);
        } catch (Exception e)
        {
        throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);
    }

    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg){
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maxSpeed);
    }

    public void drive(Translation2d translation, double rot, boolean fieldrelative){
        double smoothingFactor = 0.15;
        adjustedTranslation2d = adjustedTranslation2d.times(smoothingFactor).plus(translation.times((1 - smoothingFactor)));
        if(adjustedTranslation2d.getNorm() < 0.1){
            adjustedTranslation2d = new Translation2d(0,0);
        }
        // double desiredSpeed = adjustedTranslation2d.getNorm();
        // if(desiredSpeed > maxSpeed){
        //     double scaleFactor = maxSpeed / desiredSpeed;
        //     double scaledX = adjustedTranslation2d.getX() * scaleFactor;
        //     double scaledY = adjustedTranslation2d.getY() * scaleFactor;
        //     adjustedTranslation2d = new Translation2d(scaledX, scaledY);
        // }
        swerveDrive.drive(translation, rot, fieldrelative, false);
    }

    public SwerveController getSwerveController()
    {
        return swerveDrive.swerveController;
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public void resetPose(Pose2d HolonomicPose){
        swerveDrive.resetOdometry(HolonomicPose);
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getRawTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return swerveDrive.getRobotVelocity();
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration(){
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public Command followPathCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(0.0020645, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.005, 0.0, 0.0), // Rotation PID constants
                        2, // Max module speed, in m/s
                        0.32, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

}
