// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;


public class Climb extends Command {
  /** Creates a new Climb. */
  private Elevator elevator;

  private XboxController joyStick;

  private MoveWristToPosition moveUp;

  private final double maxInput = 0.50;

  private DigitalInput magneticLimit;

  private double initialPosition;

  public Climb(Elevator elevator, XboxController joyStick) {
    this.joyStick = joyStick;
    this.elevator = elevator;
    magneticLimit = new DigitalInput(7);

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = elevator.getElevatorEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightJoy = joyStick.getRightY();
    double speed = MathUtil.applyDeadband(rightJoy, 0.15);

    if (elevator.getClimbMode()){
      double curr = elevator.getElevatorEncoder();


      if(curr > initialPosition - IntakeConstants.MAX_DISTANCE && speed < 0){ // Go up if the current pos is less than max height and joy is up
        elevator.raiseElevatorwithSpeed(speed * maxInput);
        // System.out.println("Going up");
      }
      else if(curr < initialPosition && speed > 0 && magneticLimit.get()) { // Go down if current pos is greater than initial position (minimum height) and joy is down
        elevator.raiseElevatorwithSpeed(speed * maxInput);
        // System.out.println("Going down");
      }
      else {
        elevator.holdPosition();
      }
    }

    SmartDashboard.putNumber("Climb Position", elevator.getElevatorEncoder());
    
    // SmartDashboard.putNumber("Init pos", initialPosition);
    // SmartDashboard.putNumber("Position", elevator.getElevatorEncoder());
    // SmartDashboard.putNumber("Top pos", initialPosition + IntakeConstants.MAX_DISTANCE);
    // SmartDashboard.putBoolean("Magnetic Encoder", magneticLimit.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
