// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.Load;
import frc.robot.commands.MoveWristPercent;
import frc.robot.commands.MoveWristToPosition;
import frc.robot.commands.RumbleWhenNote;
import frc.robot.commands.ToggleClimbMode;
import frc.robot.commands.Autos.Autos;
import frc.robot.commands.Drive.DriveWithJoystick;
import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final JoystickButton xButton;
  
  private SwerveDrive swerve;
  
  private XboxController driver;
  private XboxController operator;

  private DriveWithJoystick driveWithJoystick;

  private MoveWristToPosition moveWristDown;
  private MoveWristToPosition moveWristUp;
  private MoveWristToPosition moveWristAmp;
  private SequentialCommandGroup wristDownIntake;

  private ToggleClimbMode toggleClimbMode;

  private Climb climb;

  private Outtake shootVelocity;

  private Load load;
  private Outtake outtake;

  private JoystickButton toggleFieldOrientedBtn;
  private JoystickButton toggleSlowModeBtn;
  private JoystickButton outtakeNoteBtn;
  private JoystickButton wristButton;
  private JoystickButton intakeBtn;
  private JoystickButton softOuttakeBtn;

  private JoystickButton climbButton;

  private POVButton wristDownBtn;
  private POVButton wristUpBtn;
  private POVButton wristRightBtn;
  private POVButton wristLeftBtn;

  private Intake intake;
  private Wrist wrist;
  private JoystickButton loadButton;
  private Elevator elevator;

  private MoveWristPercent moveWristPercent;
  private RumbleWhenNote rumbleWhenNote;

  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Subsystems
    intake = new Intake();
    outtake = new Outtake();
    elevator = new Elevator();
    wrist = new Wrist();
    swerve = new SwerveDrive();

    // Xbox Controllers
    driver = new XboxController(0);
    operator = new XboxController(1);

    // Intake Buttons
    xButton = new JoystickButton(operator, XboxController.Button.kX.value);
    wristButton = new JoystickButton(operator, XboxController.Button.kY.value);
    loadButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    intakeBtn = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    outtakeNoteBtn = new JoystickButton(operator, XboxController.Button.kA.value);
    softOuttakeBtn = new JoystickButton(operator, XboxController.Axis.kLeftTrigger.value);

    // Climb Buttons
    climbButton = new JoystickButton(operator, XboxController.Button.kStart.value);
    toggleClimbMode = new ToggleClimbMode(wrist, intake, elevator);

    // Driver-assisted Buttons
    wristDownBtn = new POVButton(operator, 180);
    wristUpBtn = new POVButton(operator, 0);
    wristRightBtn = new POVButton(operator, 270);
    wristLeftBtn = new POVButton(operator, 90);

    // Toggle Buttons
    toggleFieldOrientedBtn = new JoystickButton(driver, XboxController.Button.kA.value);
    toggleSlowModeBtn = new JoystickButton(driver, XboxController.Button.kX.value);
    
    driveWithJoystick = new DriveWithJoystick(swerve, driver);

    load = new Load(outtake, intake);
    climb = new Climb(elevator, operator);

    moveWristDown = new MoveWristToPosition(wrist, intake, IntakeConstants.LOW_WRIST_POS);
    moveWristUp = new MoveWristToPosition(wrist, intake, IntakeConstants.HIGH_WRIST_POS);
    moveWristAmp = new MoveWristToPosition(wrist, intake, IntakeConstants.AMP_POS);

    wristDownIntake = new SequentialCommandGroup(moveWristDown, intake.spinIntake().until(() -> !intake.getIntakeSensor()));
    moveWristPercent = new MoveWristPercent(operator, wrist);
    rumbleWhenNote = new RumbleWhenNote(intake, operator);

    autoChooser = new SendableChooser<>();
    
    wrist.setDefaultCommand(moveWristPercent);
    swerve.setDefaultCommand(driveWithJoystick);
    elevator.setDefaultCommand(climb);

    autoChooser.addOption("Middle Two Piece Speaker", Autos.MiddleTwoShoot(swerve, outtake, intake, wrist));
    autoChooser.addOption("Middle Speaker", Autos.MiddleShoot(swerve, outtake, intake));

    autoChooser.addOption("Right Side Two Piece", Autos.RightTwoPieceAuto(swerve, outtake, intake, wrist));
    autoChooser.addOption("Right Side Speaker Piece", Autos.RightAuto(swerve, outtake, intake, wrist));
    
    autoChooser.addOption("Left Side Speaker Piece", Autos.LeftAuto(swerve, outtake, intake, wrist));
    autoChooser.addOption("Left Side Two Piece", Autos.LeftTwoAuto(swerve, outtake, intake, wrist));

    autoChooser.addOption("Do nothing", Autos.DoNothing());
    autoChooser.setDefaultOption("Default", Autos.DoNothing());
   
   SmartDashboard.putData(swerve);
   SmartDashboard.putData(outtake);
   SmartDashboard.putData(intake);
   SmartDashboard.putData(wrist);
   SmartDashboard.putData(elevator);
   SmartDashboard.putData(autoChooser);
    configureBindings();
  } 

  public void setBrakeMode() {
    swerve.setBrakeMode();
  }

  public void setCoastMode() {
    swerve.setCoastMode();
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
    xButton.whileTrue(intake.outtake()); // X
    softOuttakeBtn.whileTrue(intake.softOuttake());
    loadButton.whileTrue(load); // B
    intakeBtn.whileTrue(intake.spinIntake()); // A
    wristDownBtn.onTrue(wristDownIntake); // Down on D-Pad
    wristUpBtn.onTrue(moveWristUp); // Up on D-Pad
    wristLeftBtn.onTrue(moveWristAmp); // Right on D-Pad
    wristRightBtn.onTrue(moveWristAmp); // Left on D-Pad

    climbButton.whileTrue(toggleClimbMode);

    toggleFieldOrientedBtn.whileTrue(swerve.toggleFieldOriented());
    toggleSlowModeBtn.whileTrue(swerve.toggleSlowMode());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
