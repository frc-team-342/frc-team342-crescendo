// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.AnalogInput;

import static frc.robot.Constants.IntakeConstants.HIGHWRISTPOS;
import static frc.robot.Constants.IntakeConstants.LOWWRISTPOS;
import static frc.robot.Constants.IntakeConstants.WRISTSPEED;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveWristToPosition extends Command {
  /** Creates a new MoveWristToPosition. */
  private Intake intake;
  private XboxController joyStick;
  private boolean goingDown = true;

  public MoveWristToPosition(Intake intake) {
     
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    boolean goingDown = true;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    //to make sure the wrist is not going too low becase if it did the wrist being too low could cause a motor heatup
    if (goingDown && intake.getthroughBore().getAbsolutePosition() > LOWWRISTPOS) {
      intake.rotateWrist(-WRISTSPEED);
    }
    //makes sure that its not going too far back to avoid hitting the back
    else if (!goingDown && intake.getthroughBore().getAbsolutePosition() < HIGHWRISTPOS){
      intake.rotateWrist(WRISTSPEED);
    }
    
    //lets the robot know so it wont go too far back and it knows its limits
    if (intake.getthroughBore().getAbsolutePosition() >= HIGHWRISTPOS) {
      goingDown = true;
    }
    //so it does not go too far down - sets parameters
    else if (intake.getthroughBore().getAbsolutePosition() < LOWWRISTPOS){
      goingDown = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
