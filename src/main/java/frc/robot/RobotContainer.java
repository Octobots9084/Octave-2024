// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private final SwerveSubsystem subsystemBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve"));

  public RobotContainer() {    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("1m");
    return AutoBuilder.followPath(path);
  }
}
