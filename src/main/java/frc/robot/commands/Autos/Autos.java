// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import static frc.robot.Constants.DriveConstants.MAX_DRIVE_SPEED;
import static frc.robot.Constants.IntakeConstants.HIGH_WRIST_POS;
import static frc.robot.Constants.IntakeConstants.LOW_WRIST_POS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Outtake.OuttakeNote;
import frc.robot.commands.Load;
import frc.robot.commands.MoveWristToPosition;
import frc.robot.commands.RotateToAngle;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public final class Autos {
  /** Example static factory for an autonomous command. */

public static Command shootAndScoot(SwerveDrive swerve, Outtake outtake, Intake intake, ChassisSpeeds chassisSpeeds){
  
  return Commands.sequence(

    //move Towards the speaker 
    new TimedDrive(swerve, 1.2, chassisSpeeds, MAX_DRIVE_SPEED),
    
    //Shoots the note into Speaker 
    new Load(outtake, intake).withTimeout(2),

    //Drives out of the wing 
    new TimedDrive(swerve, .5,chassisSpeeds, MAX_DRIVE_SPEED)
    
  );

}

public static Command LeftAuto(SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){

  return Commands.sequence(

  //Shoots preloaded note 
  new Load(outtake, intake),

  //Rotates the robot back to staright 
  new RotateToAngle(0, swerve),


    new ParallelCommandGroup(

    //Drives back to pick up the note 

      //new TimedDrive(swerve, .2, .2, .2),

    //Puts wrist down in order to pick up

      new MoveWristToPosition(wrist, intake, LOW_WRIST_POS)
  ),

    new ParallelCommandGroup(


    //Drives back to speaker 
    
      //new TimedDrive(swerve,.2,0,0),

    // Moves Wrist up

      new MoveWristToPosition(wrist,intake, HIGH_WRIST_POS)
  ),
  
  // Rotates robot to speaker 

  new RotateToAngle(60, swerve),

  // Shoots other note 
  
  new Load(outtake, intake)
  );
}
public static Command RightAuto (SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist, ChassisSpeeds chassisSpeeds){


  return Commands.sequence(

  new TimedDrive(swerve,1,chassisSpeeds,MAX_DRIVE_SPEED),

  new RotateToAngle(56.6, swerve),

  new Load(outtake, intake).withTimeout(3)

 // new RotateToAngle(56.6, swerve),

   // new ParallelCommandGroup(

     // new TimedDrive(swerve, .1, chassisSpeeds, MAX_DRIVE_SPEED),

     // new MoveWristToPosition(wrist,intake, LOW_WRIST_POS)
    
 // ),

   //   new TimedDrive(swerve,.1,chassisSpeeds,MAX_DRIVE_SPEED),

 // new Load(outtake, intake)

  );
}

public static Command DoNothing(){

  return Commands.none();
}

public static Command MiddleShoot(SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){

  return Commands.sequence(

  new Load(outtake, intake),

  new ParallelCommandGroup(
    
  //new TimedDrive(swerve, 2, .2, 0),

  new MoveWristToPosition(wrist, intake, LOW_WRIST_POS)
  
  ),

 // new TimedDrive(swerve, 2, .2, 0),

  new MoveWristToPosition(wrist,intake,LOW_WRIST_POS),

    new ParallelCommandGroup(
      
     new MoveWristToPosition(wrist, intake, HIGH_WRIST_POS)

      //new TimedDrive(swerve, .2, -.2, 0)
    ),

  new Load(outtake, intake)

  //new TimedDrive(swerve, .2, -.2, 0)

  );
}

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}