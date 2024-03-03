// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.HIGH_WRIST_POS;
import static frc.robot.Constants.IntakeConstants.LOW_WRIST_POS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.Outtake.OuttakeNote;
import frc.robot.commands.RotateToAngle;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public final class Autos {
  /** Example static factory for an autonomous command. */

private static Command shootAndScoot(SwerveDrive swerve, Outtake outtake, Intake intake){
  
  return Commands.sequence(

    //Shoots the note into Speaker 
    new Load(outtake, intake),

    //Drives out of the wing 
    new DriveDistance(-1, 0.2 , swerve)
    
  );

}

private static Command leftAuto(SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){

  return Commands.sequence(

  //Shoots preloaded note 
  new Load(outtake, intake),

  //Rotates the robot back to staright 
  new RotateToAngle(0, swerve),


    new ParallelCommandGroup(

    //Drives back to pick up the note 

      new TimedDrive(swerve, .2, .2, .2),

    //Puts wrist down in order to pick up

      new MoveWristToPosition(wrist, LOW_WRIST_POS)
    
  ),

    new ParallelCommandGroup(


    //Drives back to speaker 
    
      new TimedDrive(swerve,.2,0,0),

      new MoveWristToPosition(wrist, HIGH_WRIST_POS)


  ),
  

  new RotateToAngle(60, swerve),

  new Load(outtake, intake)

  );
}

private static Command rightAuto (SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){


  return Commands.sequence(

  new Load(outtake, intake),

  new RotateToAngle(0, swerve),

    new ParallelCommandGroup(

      new TimedDrive(swerve, .2, .2, .2),

      new MoveWristToPosition(wrist, LOW_WRIST_POS)
    
  ),

  new TimedDrive(swerve,.2,0,0),

  new Load(outtake, intake)

  );
}

private static Command doNothing(){

  return Commands.none();

}

private static Command middleShoot(SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){

  return Commands.sequence(

  new Load(outtake, intake),

  new ParallelCommandGroup(
    
  new TimedDrive(swerve, 2, .2, 0),

  new MoveWristToPosition(wrist, LOW_WRIST_POS)
  
  
  
  ),


  new TimedDrive(swerve, 2, .2, 0),

  new MoveWristToPosition(wrist,LOW_WRIST_POS),

    new ParallelCommandGroup(
      
     new MoveWristToPosition(wrist, HIGH_WRIST_POS),

      new TimedDrive(swerve, .2, -.2, 0)
    
    ),

  new Load(outtake, intake)

  );
}

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}