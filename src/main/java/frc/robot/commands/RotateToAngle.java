// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.DriveConstants.*;

public class RotateToAngle extends Command {

  private SwerveDrive swerve; 
  private  PIDController rotateController; 
  
  private Rotation2d start; 
  private Rotation2d end; 
  
  private Rotation2d angle;


  /** Creates a new RotateToAngle. */

  public RotateToAngle( Rotation2d angle, SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.angle = angle;
    this.swerve = swerve; 
    addRequirements(swerve);



    rotateController = new PIDController(
     
    0,
    0, 
    0

    );

    rotateController.setTolerance(.15);
    


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    start = swerve.getGyro().getRotation2d();

    end = start.plus(angle);

    rotateController.reset();

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Rotation2d current = swerve.getGyro().getRotation2d();

    double rotationSpeed = rotateController.calculate(current.getRadians(), end.getRadians());

    ChassisSpeeds radial = new ChassisSpeeds(0, 0, -rotationSpeed);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerve.getModuleStates(), MAX_ROTATE_SPEED);
   
    swerve.drive(radial);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
    swerve.stopModules();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotateController.atSetpoint();
  }
}
