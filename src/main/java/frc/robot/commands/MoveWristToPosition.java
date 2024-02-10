// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.AnalogInput;

import static frc.robot.Constants.IntakeConstants.HIGHWRISTPOS;
import static frc.robot.Constants.IntakeConstants.LOWWRISTPOS;
import static frc.robot.Constants.IntakeConstants.wristSpeed;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveWristToPosition extends Command {
  /** Creates a new MoveWristToPosition. */
  private Intake intake;
  private XboxController joyStick;
  private boolean goingDown = true;

  public MoveWristToPosition() {
     
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

    if (goingDown && intake.getthroughBore().getPosition() > lowWristPos) {
      wrist.set(LOWWRISTPOS);
    }

    else if (!goingDown && intake.getthroughBore().getPosition() < highWristPos){
      wrist.set(HIGHWRISTPOS);
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
