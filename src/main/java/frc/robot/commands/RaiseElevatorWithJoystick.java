// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.MathUtil;



public class RaiseElevatorWithJoystick extends Command {

  private Elevator elevator;
  private XboxController joyStick;


  /** Creates a new RaiseElevatorWithJoystick. */
  public RaiseElevatorWithJoystick(Elevator elevator, XboxController joyStick) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.elevator = elevator;
    this.joyStick = joyStick;

    addRequirements(elevator);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = joyStick.getLeftY();
    speed = MathUtil.applyDeadband(speed, 0.15);
    System.out.println(speed);
    //speed (-) moves up
    if (speed > 0 && elevator.getElevatorSwitchHigh() || speed <= 0 && elevator.getElevatorSwitchLow()){
      elevator.raiseElevatorwithSpeed(speed);
    }
    else{
      elevator.raiseElevatorToPosition(elevator.getElevatorEncoder());
    }
   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("At end");
    elevator.raiseElevatorToPosition(elevator.getElevatorEncoder());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
