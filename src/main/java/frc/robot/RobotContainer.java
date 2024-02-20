// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Load;
import frc.robot.commands.MoveWristToPosition;
import frc.robot.commands.OperatorCommands;
import frc.robot.commands.Drive.DriveWithJoystick;
import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.IntakeConstants.feedShooterSpeed;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.SwerveDrive;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final JoystickButton xButton;
  private final JoystickButton aButton;
  

  private SwerveDrive swerve;
  // private Outtake outtake;
  
  private XboxController joy;

  private DriveWithJoystick driveWithJoystick;
  private MoveWristToPosition moveWrist;

  private Outtake shootVelocity;

  private Load load;
  private Outtake outtake;

  private JoystickButton toggleFieldOrientedBtn;
  private JoystickButton toggleSlowModeBtn;
  private JoystickButton outtakeNoteBtn;
  private JoystickButton wristButton;
  private Intake intake;
  private JoystickButton loadButton;


  private OperatorCommands opCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    intake = new Intake();
    outtake = new Outtake();
    load = new Load(outtake, intake);
    
   //swerve = new SwerveDrive();
   //joy is xboxcontroller

    joy = new XboxController(0);
    xButton = new JoystickButton(joy, XboxController.Button.kX.value);
    aButton = new JoystickButton(joy, XboxController.Button.kA.value);
    wristButton = new JoystickButton(joy, XboxController.Button.kY.value);
    loadButton = new JoystickButton(joy, XboxController.Button.kB.value);

    //commented  out swerve to test load
   // driveWithJoystick = new DriveWithJoystick(swerve, joy, swerve.getFieldOriented());
    //moveWrist = new MoveWristToPosition(intake);
    
   // opCommands = new OperatorCommands(intake, joy);

   // intake.setDefaultCommand(opCommands);

    
   // SmartDashboard.putData(swerve);
   configureBindings();
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
   //xButton.whileTrue(intake.spinIntake());
   //aButton.whileTrue(intake.getSensors());
   //wristButton.whileTrue(moveWrist);

   loadButton.whileTrue(load);
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