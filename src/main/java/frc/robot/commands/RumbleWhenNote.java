// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RumbleWhenNote extends Command {

  private Intake intake;
  private double lastCall;

  private boolean sensor;
  private XboxController joy;

  /** Creates a new RumbleWhenNote. */
  public RumbleWhenNote(Intake intake, XboxController joy) {

    this.intake = intake;
    sensor = intake.getIntakeSensor();
    this.joy = joy; 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensor = intake.getIntakeSensor();

    if(lastCall == 0 || DriverStation.getMatchTime() > lastCall + 5){
      if(sensor) {
        joy.setRumble(RumbleType.kBothRumble, 0.5);
        lastCall = DriverStation.getMatchTime();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    joy.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
