//Help, I'm trapped in the computer only you can help me.

//Press Alt + F4 and you'll free me from my binary enclosure.

//Help me Cadence, you're my only hope!

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
import frc.robot.commands.Load;
import frc.robot.commands.MoveWristToPosition;
import frc.robot.commands.RotateToAngle;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

public final class Autos {
  /** Example static factory for an autonomous command. */

public static Command MiddleShoot(SwerveDrive swerve, Outtake outtake, Intake intake) {

  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1,0,0);

  return Commands.sequence(

    //move Towards the speaker 
    new TimedDrive(swerve, 1, chassisSpeeds, MAX_DRIVE_SPEED),
    
    //Shoots the note into Speaker 
    new Load(outtake, intake).withTimeout(2),

    //Drives out of the wing 
    new TimedDrive(swerve, .6,chassisSpeeds, MAX_DRIVE_SPEED)
    
  );

}

public static Command MiddleTwoShoot(SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){
  
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1,0,0);
  ChassisSpeeds negChassisSpeeds = new ChassisSpeeds(-1,0,0);

  return Commands.sequence(

    //move Towards the speaker 
    new TimedDrive(swerve, 1, chassisSpeeds, MAX_DRIVE_SPEED),
    
    //Shoots the note into Speaker 
    new Load(outtake, intake).withTimeout(2),

    // Backs up to give room for intake
    new TimedDrive(swerve, 0.7, negChassisSpeeds, MAX_DRIVE_SPEED),

    // Moves wrist down
    new MoveWristToPosition(wrist, intake, LOW_WRIST_POS),
    new TimedDrive(swerve, 0.9, chassisSpeeds, MAX_DRIVE_SPEED),
    
    // Moves back and readies for load
    new ParallelCommandGroup(  
      new TimedDrive(swerve, 0.6, negChassisSpeeds, MAX_DRIVE_SPEED),
      new MoveWristToPosition(wrist, intake, HIGH_WRIST_POS)),
    
    new Load(outtake, intake).withTimeout(2),
    new TimedDrive(swerve, 1, chassisSpeeds, MAX_DRIVE_SPEED)
  ); 
}

public static Command LeftAuto(SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){

  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1,0,0);

  if(swerve.shouldFlip()){  
    return Commands.sequence(
    new TimedDrive(swerve, 1, chassisSpeeds , MAX_DRIVE_SPEED),
    new RotateToAngle(50.3, swerve).withTimeout(2),
    
    //Shoots preloaded note 
    new Load(outtake, intake).withTimeout(2),

    //Rotates the robot back to straight 
    new RotateToAngle(0, swerve).withTimeout(2),
    new MoveWristToPosition(wrist, intake, LOW_WRIST_POS),
    new TimedDrive(swerve, 2, chassisSpeeds, MAX_DRIVE_SPEED),
    new MoveWristToPosition(wrist, intake, HIGH_WRIST_POS)
    );
  }

  return Commands.sequence(
  new TimedDrive(swerve, 1, chassisSpeeds , MAX_DRIVE_SPEED),
  new RotateToAngle(-50.3, swerve).withTimeout(2),
  
  //Shoots preloaded note 
  new Load(outtake, intake).withTimeout(2),

  //Rotates the robot back to straight 
  new RotateToAngle(0, swerve).withTimeout(2),
  new MoveWristToPosition(wrist, intake, LOW_WRIST_POS),
  new TimedDrive(swerve, 2, chassisSpeeds, MAX_DRIVE_SPEED),
  new MoveWristToPosition(wrist, intake, HIGH_WRIST_POS)
  );
}

public static Command LeftTwoAuto(SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){

  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1,0,0);
  ChassisSpeeds negChassisSpeeds = new ChassisSpeeds(-1,0,0);

  if(swerve.shouldFlip()){  
    return Commands.sequence( // Red Left
      new TimedDrive(swerve,1,chassisSpeeds,MAX_DRIVE_SPEED), // Drive out
      new RotateToAngle(-55, swerve).withTimeout(2), // Rotate to shoot position
      new Load(outtake, intake).withTimeout(1.5), // Shoot

      new ParallelCommandGroup( // Rotate toward note while lowering wrist
        new RotateToAngle(12, swerve), 
        new MoveWristToPosition(wrist, intake, LOW_WRIST_POS)),
      
      new TimedDrive(swerve, 1.3, chassisSpeeds, MAX_DRIVE_SPEED), // Drive to note and pick it up
      new MoveWristToPosition(wrist, intake, HIGH_WRIST_POS), // Prep note for shooting

      new TimedDrive(swerve,1.3, negChassisSpeeds,MAX_DRIVE_SPEED), // Drive back to speaker
      new RotateToAngle(-55, swerve).withTimeout(2), // Rotate to shoot position
      new Load(outtake, intake).withTimeout(2), // Shoot
      new RotateToAngle(-20, swerve).withTimeout(2), // Rotate to leave position
      new TimedDrive(swerve, 1, new ChassisSpeeds(2.5,0,0), MAX_DRIVE_SPEED)); // Leave
  }

  return Commands.sequence( // Blue Left
    new TimedDrive(swerve, 1.3, chassisSpeeds, MAX_DRIVE_SPEED), // Drive out
    new RotateToAngle(-43, swerve).withTimeout(1.5), // Rotate to shoot position
    new Load(outtake, intake).withTimeout(1.5), // Shoot

    new ParallelCommandGroup( // Rotate toward note while lowering wrist
      new RotateToAngle(0, swerve).withTimeout(2),
      new MoveWristToPosition(wrist, intake, LOW_WRIST_POS)),

    new TimedDrive(swerve, 1, chassisSpeeds, MAX_DRIVE_SPEED), // Drive to note and pick it up
    new MoveWristToPosition(wrist, intake, HIGH_WRIST_POS), // Prep note for shooting

    new TimedDrive(swerve, 1, chassisSpeeds, MAX_DRIVE_SPEED), // Leave
    new TimedDrive(swerve, 0.2, new ChassisSpeeds(0, -1,0), MAX_DRIVE_SPEED), // Shuffle right
    new TimedDrive(swerve, 2, negChassisSpeeds, MAX_DRIVE_SPEED), // Drive back to speaker

    new RotateToAngle(-43, swerve).withTimeout(2), // Rotate to shoot position
    new Load(outtake, intake).withTimeout(2) // Shoot
  );
}

public static Command RightTwoPieceAuto (SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){
  
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1,0,0);
  ChassisSpeeds negChassisSpeeds = new ChassisSpeeds(-1, 0, 0);
  
  if(swerve.shouldFlip()) {
   return Commands.sequence(

  //drive out
  new TimedDrive(swerve,1,chassisSpeeds,MAX_DRIVE_SPEED),

  //line up shot
  new RotateToAngle(-55, swerve).withTimeout(2),
 
  //shoot
  new Load(outtake, intake).withTimeout(1.5),

  //rotate and lower wrist
  new ParallelCommandGroup(
    new RotateToAngle(12, swerve), 
    new MoveWristToPosition(wrist, intake, LOW_WRIST_POS)),
  
  //drive to note
  new TimedDrive(swerve, 1.3, chassisSpeeds, MAX_DRIVE_SPEED),
  
  new MoveWristToPosition(wrist, intake, HIGH_WRIST_POS),

  new TimedDrive(swerve,1.3, negChassisSpeeds,MAX_DRIVE_SPEED),

  new RotateToAngle(-55, swerve).withTimeout(2),
  new Load(outtake, intake).withTimeout(2),
  new RotateToAngle(-20, swerve).withTimeout(2),
  new TimedDrive(swerve, 1.5, new ChassisSpeeds(2.5,0,0), MAX_DRIVE_SPEED));
  }

  return Commands.sequence(

  //drive out
  new TimedDrive(swerve,1,chassisSpeeds,MAX_DRIVE_SPEED),

  //line up shot
  new RotateToAngle(55, swerve).withTimeout(2),
 
  //shoot
  new Load(outtake, intake).withTimeout(1.5),

  //rotate and lower wrist
  new ParallelCommandGroup(
    new RotateToAngle(-12, swerve), 
    new MoveWristToPosition(wrist, intake, LOW_WRIST_POS)),
  
  //drive to note
  new TimedDrive(swerve, 1.3, chassisSpeeds, MAX_DRIVE_SPEED),
  
  new MoveWristToPosition(wrist, intake, HIGH_WRIST_POS),

  new TimedDrive(swerve,1.3, negChassisSpeeds,MAX_DRIVE_SPEED),

  new RotateToAngle(55, swerve).withTimeout(2),
  new Load(outtake, intake).withTimeout(2),
  new RotateToAngle(20, swerve).withTimeout(2),
  new TimedDrive(swerve, 1, new ChassisSpeeds(2.5,0,0), MAX_DRIVE_SPEED));
  }

public static Command RightAuto (SwerveDrive swerve, Outtake outtake, Intake intake, Wrist wrist){

  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1,0,0);

  if(swerve.shouldFlip()) {
   return Commands.sequence(
    new TimedDrive(swerve,1,chassisSpeeds,MAX_DRIVE_SPEED),
    new RotateToAngle(-50.3, swerve).withTimeout(2),
    //subtacrted 5 from the angle to account for overshoot
    new Load(outtake, intake).withTimeout(3),
    new RotateToAngle(0, swerve).withTimeout(2),
    new TimedDrive(swerve, 1.1, chassisSpeeds, MAX_DRIVE_SPEED)); 
    }

  return Commands.sequence(

  new TimedDrive(swerve,1,chassisSpeeds,MAX_DRIVE_SPEED),

  new RotateToAngle(50.3, swerve).withTimeout(2),
  new Load(outtake, intake).withTimeout(2),

  new RotateToAngle(0, swerve).withTimeout(2),

  new TimedDrive(swerve, 1.1, chassisSpeeds, MAX_DRIVE_SPEED)
  );
}

public static Command DoNothing(){

  return Commands.none();
}

public static Command AmpAuto(SwerveDrive swerve){
  return Commands.sequence(
    new TimedDrive(swerve, 1.5, new ChassisSpeeds(1,0,0), MAX_DRIVE_SPEED),
    new TimedDrive(swerve, 0.5, new ChassisSpeeds(0,-1,0), MAX_DRIVE_SPEED),
    swerve.rotateToAmp(),
    new TimedDrive(swerve, 0.5, new ChassisSpeeds(1,0,0), MAX_DRIVE_SPEED)
  );
}

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}