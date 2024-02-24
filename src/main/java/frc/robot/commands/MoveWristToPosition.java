// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.AnalogInput;

import static frc.robot.Constants.IntakeConstants.*;

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
    if (goingDown && intake.getthroughBore().getPosition() > LOW_WRIST_POS) {
      intake.rotateWrist(-WRIST_SPEED);
    }
    //makes sure that its not going too far back to avoid hitting the back
    else if (!goingDown && intake.getthroughBore().getPosition() < HIGH_WRIST_POS){
      intake.rotateWrist(WRIST_SPEED);
    }
    
    //lets the robot know so it wont go too far back and it knows its limits
    if (intake.getthroughBore().getPosition() >= HIGH_WRIST_POS) {
      goingDown = true;
    }
    //so it does not go too far down - sets parameters
    else if (intake.getthroughBore().getPosition() < LOW_WRIST_POS){
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
