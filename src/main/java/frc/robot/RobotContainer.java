// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.OperatorCommands;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //xxcontroller - xbox controller
    private final XboxController xxcontroller; 
    private final JoystickButton xButton;
    private final JoystickButton aButton;

    private final Intake intake;


    private final OperatorCommands opCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    intake = new Intake();
   // swerve = new SwerveDrive();

    xxcontroller = new XboxController(0);
    xButton = new JoystickButton(xxcontroller, XboxController.Button.kX.value);
    aButton = new JoystickButton(xxcontroller, XboxController.Button.kA.value);
    
    opCommands = new OperatorCommands(intake, xxcontroller);

    intake.setDefaultCommand(opCommands);

    SmartDashboard.putData(intake);
   // SmartDashboard.putData(swerve);
  } 


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
 

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    
   xButton.whileTrue(intake.spinIntake());
   //aButton.whileTrue(intake.getSensors());


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   return null;
    
  }
}
