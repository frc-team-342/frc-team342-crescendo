// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter rotateLimiter;

  private ChassisSpeeds chassisSpeeds;
  private SwerveModuleState[] moduleStates;
  private SwerveDriveKinematics swerveKinematics;

  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(SwerveDrive swerve, XboxController joy) {

    this.swerve = swerve;
    this.joy = joy;

    fieldOriented = swerve.getFieldOriented();

    xLimiter = new SlewRateLimiter(3);
    yLimiter = new SlewRateLimiter(3);
    rotateLimiter = new SlewRateLimiter(3);

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

    if(fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotateSpeed, swerve.getGyro().getRotation2d());
      System.out.println("Driving field oriented");
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed);
      System.out.println("Driving robot oriented");
    }

    moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates, maxDriveSpeed);

    SmartDashboard.putNumber("Chassis x-speed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis y-speed", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis rotate-speed", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Gyro", swerve.getGyro().getRotation2d().getRadians());
    SmartDashboard.putBoolean("Slow Mode", swerve.getSlowMode());
    SmartDashboard.putNumber("Current Max Speed", maxDriveSpeed);
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
