package frc.robot.commands.swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class TeleopDrive extends Command{
    //Variables
    private final SwerveSubsystem swerve;
    private final DoubleSupplier velocityX;
    private final DoubleSupplier velocityY;
    private final DoubleSupplier angularVelocity;
    private final BooleanSupplier drivemode;
    private final SwerveController controller;
    
    //Constructor
    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier angularVelocity, BooleanSupplier drivemode){
        this.swerve = swerve;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.angularVelocity = angularVelocity;
        this.drivemode = drivemode;
        this.controller = swerve.getSwerveController();

        this.addRequirements(swerve);
    }

    @Override
    public void initialize(){}
    @Override
    public void execute(){
        double xVelocity = Math.pow(velocityX.getAsDouble(), 3);
        double yVelocity = Math.pow(velocityY.getAsDouble(), 3);
        double angVelocity = Math.pow(angularVelocity.getAsDouble(), 3);
        
        SmartDashboard.putNumber("Angular Velocity", angVelocity);
        SmartDashboard.putNumber("X Velocity", xVelocity);
        SmartDashboard.putNumber("Y Velocity", yVelocity);

        Translation2d translation = new Translation2d(xVelocity * swerve.maxSpeed, yVelocity * swerve.maxSpeed);
        
        swerve.drive(translation, angVelocity * 0.4, drivemode.getAsBoolean());
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
 