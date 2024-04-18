// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class DriveWithJoystick extends Command {

  private SwerveDrive swerve;
  private XboxController joy;
  private AHRS gyro;

  private Supplier<Double> rotateSpeed;
  private boolean fieldOriented;
  private boolean zeroMode;
  private boolean ninetyMode;
  private boolean cameFromZero;
  private boolean cameFromNinety;
  private PIDController rotateController;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter rotateLimiter;

  public ChassisSpeeds chassisSpeeds;
  private SwerveModuleState[] moduleStates;
  private SwerveDriveKinematics swerveKinematics;
  private static double lastHeading;
  int count = 0;

  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(SwerveDrive swerve, XboxController joy, boolean zeroMode, boolean ninetyMode) {

    this.swerve = swerve;
    this.joy = joy;

    fieldOriented = swerve.getFieldOriented();
    this.zeroMode = zeroMode;
    this.ninetyMode = ninetyMode;
    
    rotateController = new PIDController(0.1, 0, 0);
    rotateController.setTolerance(1);

    xLimiter = new SlewRateLimiter(3);
    yLimiter = new SlewRateLimiter(3);
    rotateLimiter = new SlewRateLimiter(3);

    rotateController.reset();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = joy.getLeftY();
    double ySpeed = joy.getLeftX();
    double rotateSpeed = joy.getRawAxis(4);
    double maxDriveSpeed = swerve.getSlowMode() ? DriveConstants.SLOWER_DRIVE_SPEED : DriveConstants.MAX_DRIVE_SPEED;

    fieldOriented = swerve.getFieldOriented();

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.15);
    rotateSpeed = MathUtil.applyDeadband(rotateSpeed, 0.15);

    xSpeed = xLimiter.calculate(xSpeed) * maxDriveSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * maxDriveSpeed;
    rotateSpeed = rotateLimiter.calculate(rotateSpeed) * DriveConstants.MAX_ROTATE_SPEED;


    if(cameFromNinety){
      lastHeading = swerve.shouldFlip() ? 90 : -90;
      cameFromNinety = false;
    }else if(cameFromZero){
      lastHeading = 0;
      cameFromZero = false;
    }
    else if((Math.abs(rotateSpeed) > 0.15)){
      
        lastHeading = swerve.getGyro().getAngle();
  
    }

    if(zeroMode) {
      rotateSpeed = rotateController.calculate(swerve.getHeading(), 0);
      lastHeading = 0;
      cameFromZero = true;
    }
    else if(ninetyMode) {
      double rotation = swerve.shouldFlip() ? 90 : -90;
      rotateSpeed = rotateController.calculate(swerve.getHeading(), rotation);
      lastHeading = rotation;
      cameFromNinety = true;
    }
    else {
      //rotateSpeed += rotateController.calculate(swerve.getGyro().getAngle(), lastHeading);//trouble between -180 to 180
    
    }

   

    if(fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotateSpeed, swerve.getGyro().getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed);
    }

    moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates, maxDriveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

   // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
