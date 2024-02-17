// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.TimedDrive;
import frc.robot.commands.Drive.DriveWithJoystick;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private SwerveDrive swerve;
  
  private XboxController joy;
  private DriveWithJoystick driveWithJoystick;
  private JoystickButton toggleFieldOrientedBtn;
  private JoystickButton goToZeroBtn;
  private JoystickButton timedDriveButton;
  private JoystickButton rotateToAngleButton; 
  private TimedDrive driveFoward;
  private RotateToAngle rotate90;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    swerve = new SwerveDrive();
    joy = new XboxController(0);
    driveWithJoystick = new DriveWithJoystick(swerve, joy, swerve.getFieldOriented());
    timedDriveButton = new JoystickButton(joy,  XboxController.Button.kY.value);
    driveFoward = new TimedDrive(swerve, 2, 0, 0);
    rotate90 = new RotateToAngle( 270, swerve);
    rotateToAngleButton = new JoystickButton(joy, XboxController.Button.kB.value);

    
    swerve.setDefaultCommand(driveWithJoystick);
    toggleFieldOrientedBtn = new JoystickButton(joy, XboxController.Button.kA.value);
    goToZeroBtn = new JoystickButton(joy, XboxController.Button.kX.value);

    SmartDashboard.putData(swerve);

    // Configure the trigger bindings
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
    toggleFieldOrientedBtn.whileTrue(swerve.toggleFieldOriented());
    goToZeroBtn.whileTrue(swerve.goToZero());
    timedDriveButton.whileTrue(driveFoward);
    rotateToAngleButton.whileTrue(rotate90);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    PathPlannerAuto auto = new PathPlannerAuto("Basic Auto");
    System.out.println("Got here");
    return auto;
  }
}
