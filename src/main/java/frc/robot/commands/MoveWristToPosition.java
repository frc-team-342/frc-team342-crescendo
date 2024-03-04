// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.AnalogInput;

import static frc.robot.Constants.IntakeConstants.HIGH_WRIST_POS;
import static frc.robot.Constants.IntakeConstants.LOW_WRIST_POS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Wrist;


public class MoveWristToPosition extends Command {
  /** Creates a new MoveWristToPosition. */
  private Wrist wrist;
  private Intake intake;

  private XboxController joyStick;
  private boolean goingDown;
  private double position;

  public MoveWristToPosition(Wrist wrist, Intake intake, double position) {
     
    this.wrist = wrist;
    this.intake = intake;
    
    boolean goingDown = false;
    this.position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currPosition = wrist.getthroughBore().getAbsolutePosition();
    goingDown = currPosition < position;

    //to make sure the wrist is not going too low becase if it did the wrist being too low could cause a motor heatup
    if (goingDown && currPosition < LOW_WRIST_POS) {
      wrist.rotateWrist(-.75);
      // System.out.println("Moving Down");
    }
    //makes sure that its not going too far back to avoid hitting the back
    else if (!goingDown && currPosition > HIGH_WRIST_POS){
      wrist.rotateWrist(.75);
      // System.out.println("Moving Up");
    }
  
   if(intake.getIntakeSensor()) {
      intake.hold();
      // System.out.println("Holding)");
    }
    else {
      intake.stop();
      // System.out.println("Stopping");
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //intake.rotateWristToPosition(position);
    wrist.rotateWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Finished moving arm");
    return (wrist.getthroughBore().getAbsolutePosition() >= position - 0.01) && (wrist.getthroughBore().getAbsolutePosition() <= position + 0.01);
  }
}
