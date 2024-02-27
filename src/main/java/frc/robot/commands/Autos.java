// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.Outtake.OuttakeNote;
import frc.robot.commands.RotateToAngle;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.RobotContainer;

public final class Autos {
  /** Example static factory for an autonomous command. */

private static Command shootAndScoot(SwerveDrive swere, Outtake outtake){
  
  return Commands.sequence(

    //Shoots the note into whatever 
    new OuttakeNote(3000, outtake)

    //Drives out of whatevr 
    new DriveDistance(1, 0.2 , swere)
    
  );

}


  


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
