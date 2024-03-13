// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleClimbMode extends SequentialCommandGroup {
  
  private Wrist wrist;
  private Intake intake;
  private Elevator elevator;

  private MoveWristToPosition moveUp;
  private Command toggleClimb;
  private Climb climb;

  /** Creates a new ToggleClimbMode. */
  public ToggleClimbMode(Wrist wrist, Intake intake, Elevator elevator) {

    this.wrist = wrist;
    this.intake = intake;
    this.elevator = elevator;

    moveUp = new MoveWristToPosition(wrist, intake, IntakeConstants.HIGH_WRIST_POS);
    toggleClimb = elevator.toggleClimbMode();

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(toggleClimb, moveUp);
  }
}
