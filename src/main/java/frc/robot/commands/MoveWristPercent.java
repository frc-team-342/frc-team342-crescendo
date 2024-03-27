// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Wrist;

public class MoveWristPercent extends Command {

private XboxController joy;
private Wrist wrist;

  /** Creates a new MoveWristPercent. */
  public MoveWristPercent(XboxController joy, Wrist wrist) {

    this.joy = joy;
    this.wrist = wrist;

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

    double speed = MathUtil.applyDeadband(joy.getLeftY(), 0.15);
    if(currPosition > IntakeConstants.HIGH_WRIST_POS && currPosition < IntakeConstants.LOW_WRIST_POS){
      wrist.rotateWrist(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.rotateWrist(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
