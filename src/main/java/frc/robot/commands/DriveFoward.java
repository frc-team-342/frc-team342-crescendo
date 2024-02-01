// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Timer;


public class DriveFoward extends Command {
  /** Creates a new DriveFoward. */

  private final Timer m_timer = new Timer();
  private SwerveDrive swere;
  private double driveTime;
  
  public DriveFoward( SwerveDrive swere, double driveTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    
      this.swere = swere; 
      this.driveTime = driveTime; 
      addRequirements(swere);



  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_timer.restart();
    System.out.println("Initializing Timer");

  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {

    System.out.println("Timer: " + m_timer.get() + " ");
    swere.drive(0, .2, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swere.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > driveTime;
  }
}
