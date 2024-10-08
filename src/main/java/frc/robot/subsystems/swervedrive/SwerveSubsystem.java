
package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private Rotation2d shootingRequest = new Rotation2d();
  private boolean shootingRequestActive = false;

  private Pose2d alignRequest = new Pose2d();
  private boolean alignRequestActive = false;

  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  private static SwerveSubsystem swerveSubsystem = null;
  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public static final double MAXIMUM_SPEED = 5;
  public Rotation2d targetAngle = new Rotation2d();
  public boolean targetAngleEnabled = false;
  public PIDController targetAngleController;
  public PIDController driverTargetAngleController;
  public boolean collectAutoRunning = false;
  private Pose2d shooterPose = new Pose2d();

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  private SwerveSubsystem(File directory) {

    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.MACHINE;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(5, 360,

          SwerveMath.calculateMetersPerRotation(0.076, 40.0 / 11.0));

    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                             // angle.

    targetAngleController = Constants.Drivebase.TAREGET_ANGLE_CONTROLLER;
    ;
    targetAngleController.enableContinuousInput(-Math.PI, Math.PI);

    driverTargetAngleController = Constants.Drivebase.DRIVER_TAREGET_ANGLE_CONTROLLER;
    ;
    driverTargetAngleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static SwerveSubsystem getInstance() {
    if (swerveSubsystem == null) {
      swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
          "swerve"));
    }

    return swerveSubsystem;
  }

  public SwerveDrive getSwerveDrive() {
    return this.swerveDrive;
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(() -> {
      // Make the robot move
      driveFieldOriented(getTargetSpeeds(translationX.getAsDouble(), translationY.getAsDouble(),
          headingX.getAsDouble(),
          headingY.getAsDouble()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * MAXIMUM_SPEED, translationY.getAsDouble()),
          angularRotationX.getAsDouble() * swerveDrive.swerveController.config.maxAngularVelocity,
          true,
          false);
    });
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, MAXIMUM_SPEED);
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
  }

  public void setShootingRequest(Rotation2d target) {
    shootingRequest = target;
  }

  public void setShootingRequestActive(boolean isActive) {
    shootingRequestActive = isActive;
  }

  public boolean getShootingRequestActive() {
    return shootingRequestActive;
  }

  public Rotation2d getShootingRequest() {
    return shootingRequest;
  }

  public void setAlignRequest(Pose2d target) {
    alignRequest = target;
  }

  public void setAlignRequestActive(boolean isActive) {
    // SmartDashboard.putBoolean("Align Active", isActive);

    alignRequestActive = isActive;
  }

  public boolean getAlignRequestActive() {
    return alignRequestActive;
  }

  public Pose2d getAlignRequest() {
    return alignRequest;
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.setGyro(new Rotation3d(0, 0, initialHolonomicPose.getRotation().getRadians()));
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {

    swerveDrive.setChassisSpeeds(chassisSpeeds);

  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    if (Constants.isBlueAlliance) {
      swerveDrive.zeroGyro();
      System.out.println("Gyro zeroed on Blue side.");

    } else {
      swerveDrive.zeroGyro();
      swerveDrive.setGyro(new Rotation3d(0, 0, Math.PI));
      System.out.println("Gyro zeroed on Red side.");

    }

  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW
   * positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    double curvedXInput = Math.pow(xInput, 3);
    double curvedYInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(curvedXInput,
        curvedYInput,
        headingX,
        headingY,
        getHeading().getRadians(),
        MAXIMUM_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    double curvedXInput = xInput;
    double curvedYInput = yInput;
    return swerveDrive.swerveController.getTargetSpeeds(curvedXInput,
        curvedYInput,
        angle.getRadians(),
        getHeading().getRadians(),
        MAXIMUM_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addVisionReading(Pose2d robotPose, double timestamp,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    swerveDrive.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
  }

  public void setShooterPose(Pose2d shooterPose) {
    this.shooterPose = shooterPose;
  }

  public Pose2d getShooterPose() {
    return shooterPose;
  }
}
