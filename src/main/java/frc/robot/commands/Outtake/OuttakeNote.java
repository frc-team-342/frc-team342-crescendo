// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

public class OuttakeNote extends Command {

  private Outtake outtake;
  private double targetSpeed;

  /** Creates a new OuttakeNote. */
  public OuttakeNote(double targetSpeed, Outtake outtake) {

    this.outtake = outtake;
    this.targetSpeed = targetSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(outtake.isUpToSpeed(targetSpeed)){
      outtake.shootVelocity(targetSpeed);
    // }

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
