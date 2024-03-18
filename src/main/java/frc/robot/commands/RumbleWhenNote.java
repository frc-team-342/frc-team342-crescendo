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

  private Timer rumbleTime;
  private Timer coolDownTime;

  private boolean sensor;
  private XboxController joy;

  /** Creates a new RumbleWhenNote. */
  public RumbleWhenNote(Intake intake, XboxController joy) {

    this.intake = intake;
    sensor = intake.getIntakeSensor();
    this.joy = joy; 

    rumbleTime = new Timer();
    coolDownTime = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensor = !intake.getIntakeSensor();

   if(coolDownTime.get() > 3 && intake.isStuck()) {
        joy.setRumble(RumbleType.kBothRumble, 0.5);
        rumbleTime.start();

        coolDownTime.stop();
        coolDownTime.reset();

        System.out.println("Rumbling");
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    joy.setRumble(RumbleType.kBothRumble, 0);
    rumbleTime.stop();
    rumbleTime.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rumbleTime.get() > 3;
  }
}
