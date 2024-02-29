// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
<<<<<<< HEAD
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.Drive.DriveWithJoystick;
import frc.robot.commands.Outtake.OuttakeNote;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.SwerveDrive;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
=======
import frc.robot.commands.Load;
import frc.robot.commands.MoveWristPercent;
import frc.robot.commands.MoveWristToPosition;
import frc.robot.commands.Drive.DriveWithJoystick;
>>>>>>> da1d78a6d31e038f5f77268f33f42f8881c8589d
import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.IntakeConstants.feedShooterSpeed;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  // private final JoystickButton aButton;
  
  private SwerveDrive swerve;
  // private Outtake outtake;
  
  private XboxController driver;
  private XboxController operator;

  private DriveWithJoystick driveWithJoystick;
<<<<<<< HEAD
  private DriveDistance driveDistance;
  private OuttakeNote outtakeNote;
  private TimedDrive driveFoward;
  private RotateToAngle rotate90;
=======
  // private MoveWristToPosition moveWrist;

  private Outtake shootVelocity;

  private Load load;
  private Outtake outtake;
>>>>>>> da1d78a6d31e038f5f77268f33f42f8881c8589d

  private JoystickButton goToZeroBtn;
  private JoystickButton rotateToAngleButton;
  private JoystickButton toggleFieldOrientedBtn;
  private JoystickButton toggleSlowModeBtn;
  private JoystickButton timedDriveButton;
  private JoystickButton outtakeNoteBtn;
<<<<<<< HEAD
  private JoystickButton driveDistanceButton;
=======
  private JoystickButton wristButton;
  private JoystickButton intakeBtn;
>>>>>>> da1d78a6d31e038f5f77268f33f42f8881c8589d

  private Intake intake;
  private JoystickButton loadButton;

  private MoveWristPercent moveWristPercent;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    intake = new Intake();
    outtake = new Outtake();
    load = new Load(outtake, intake);
    
    swerve = new SwerveDrive();

<<<<<<< HEAD
    joy = new XboxController(0);
    driveWithJoystick = new DriveWithJoystick(swerve, joy);
    // outtakeNote = new OuttakeNote(0.5, outtake);
    driveDistance = new DriveDistance(1, 5, swerve);

    //driveWithJoystick = new DriveWithJoystick(swerve, joy, swerve.getFieldOriented());
    timedDriveButton = new JoystickButton(joy,  XboxController.Button.kY.value);
    driveFoward = new TimedDrive(swerve, 2, 0, 0);
    rotate90 = new RotateToAngle( 270, swerve);
    rotateToAngleButton = new JoystickButton(joy, XboxController.Button.kB.value);

    
    swerve.setDefaultCommand(driveWithJoystick);
    toggleFieldOrientedBtn = new JoystickButton(joy, XboxController.Button.kA.value);
    toggleSlowModeBtn = new JoystickButton(joy, XboxController.Button.kX.value);
    // outtakeNoteBtn = new JoystickButton(joy, XboxController.Button.kB.value);
    driveDistanceButton = new JoystickButton(joy, XboxController.Button.kY.value);

=======
    driver = new XboxController(0);
    operator = new XboxController(1);

    xButton = new JoystickButton(operator, XboxController.Button.kX.value);
    // aButton = new JoystickButton(joy, XboxController.Button.kA.value);
    wristButton = new JoystickButton(operator, XboxController.Button.kY.value);
    loadButton = new JoystickButton(operator, XboxController.Button.kB.value);
    intakeBtn = new JoystickButton(operator, XboxController.Button.kA.value);

    SmartDashboard.putData(outtake);
    SmartDashboard.putData(intake);

    driveWithJoystick = new DriveWithJoystick(swerve, driver, swerve.getFieldOriented());
    // moveWrist = new MoveWristToPosition(intake);
    moveWristPercent = new MoveWristPercent(operator, intake);
    intake.setDefaultCommand(moveWristPercent);

    outtakeNoteBtn = new JoystickButton(operator, XboxController.Button.kA.value);
    
    swerve.setDefaultCommand(driveWithJoystick);
>>>>>>> da1d78a6d31e038f5f77268f33f42f8881c8589d

   // SmartDashboard.putData(swerve);
    configureBindings();
  } 

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
<<<<<<< HEAD
    toggleFieldOrientedBtn.whileTrue(swerve.toggleFieldOriented());
    toggleSlowModeBtn.whileTrue(swerve.toggleSlowMode());
    // outtakeNoteBtn.whileTrue(outtakeNote);
    goToZeroBtn.whileTrue(swerve.goToZero());
    timedDriveButton.whileTrue(driveFoward);
    rotateToAngleButton.whileTrue(rotate90);
    
=======
   xButton.whileTrue(intake.spinIntake());
  //  aButton.whileTrue(intake.getSensors());
  //  wristButton.whileTrue(moveWrist);
   loadButton.whileTrue(load);
   intakeBtn.whileTrue(intake.spinIntake());
>>>>>>> da1d78a6d31e038f5f77268f33f42f8881c8589d
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