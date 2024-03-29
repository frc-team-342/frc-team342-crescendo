// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.RotateToAngle;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;

  private AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;
  
  private SwerveModulePosition[] positions;
  private SwerveModuleState[] states;

  private ChassisSpeeds chassisSpeeds;
  private Supplier<Pose2d> poseSupplier;
  private Consumer<Pose2d> resetPoseConsumer;
  private Consumer<ChassisSpeeds> robotRelativeOutput;
  private Supplier<ChassisSpeeds> chassisSpeedSupplier;
  private BooleanSupplier shouldFlipSupplier;
  private Field2d field;

  SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  SlewRateLimiter rotateLimiter = new SlewRateLimiter(3);

  private boolean fieldOriented;
  public boolean slowMode;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {

    frontLeft = new SwerveModule(
      DriveConstants.FRONT_LEFT[0],
      DriveConstants.FRONT_LEFT[1],
      DriveConstants.FL_ENCODER_PORT,
      false, false,
      DriveConstants.FRONT_LEFT_OFFSET,
      DriveConstants.FL_PID_VALUES);

    frontRight = new SwerveModule(
      DriveConstants.FRONT_RIGHT[0], 
      DriveConstants.FRONT_RIGHT[1],
      DriveConstants.FR_ENCODER_PORT,
      false, true,
      DriveConstants.FRONT_RIGHT_OFFSET,
      DriveConstants.FR_PID_VALUES);
   
    backLeft = new SwerveModule(
      DriveConstants.BACK_LEFT[0],
      DriveConstants.BACK_LEFT[1],
      DriveConstants.BL_ENCODER_PORT,
      false, true,
      DriveConstants.BACK_LEFT_OFFSET,
      DriveConstants.BL_PID_VALUES);  
      
    backRight = new SwerveModule(
      DriveConstants.BACK_RIGHT[0],
      DriveConstants.BACK_RIGHT[1],
      DriveConstants.BR_ENCODER_PORT,
      false, false,
      DriveConstants.BACK_RIGHT_OFFSET,
      DriveConstants.BR_PID_VALUES);

    gyro = new AHRS(SerialPort.Port.kUSB);

    states = new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };

    swerveOdometry = new SwerveDriveOdometry(KINEMATICS, new Rotation2d(gyro.getAngle()), getModulePositions());
    chassisSpeeds = new ChassisSpeeds();

    poseSupplier = () -> getPose();
    resetPoseConsumer = pose -> resetOdometry(pose);
    robotRelativeOutput = inputSpeed -> drive(inputSpeed, SLOWER_DRIVE_SPEED);
    chassisSpeedSupplier = () -> getChassisSpeeds();
    shouldFlipSupplier = () -> false;

    fieldOriented = true;
    slowMode = false;

    field = new Field2d();

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {}
    }).start();
    
    configureAutoBuilder();
  }

  public AHRS getGyro() {
    return gyro;
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  private void zeroHeading() {
    gyro.reset();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(getHeading() * (Math.PI / 180));
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Sets modules to speeds within the allowed range
   * @param desiredStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, double maxDriveSpeed) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxDriveSpeed);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    positions[0] = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getRotatePosition()));
    positions[1] = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getRotatePosition()));
    positions[2] = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getRotatePosition()));
    positions[3] = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getRotatePosition()));
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    states[0] = new SwerveModuleState(frontLeft.getDriveVelocity(), new Rotation2d(frontLeft.getRotatePosition()));
    states[1] = new SwerveModuleState(frontRight.getDriveVelocity(), new Rotation2d(frontRight.getRotatePosition()));
    states[2] = new SwerveModuleState(backLeft.getDriveVelocity(), new Rotation2d(backLeft.getRotatePosition()));
    states[3] = new SwerveModuleState(backRight.getDriveVelocity(), new Rotation2d(backRight.getRotatePosition()));
    return states;
  }

  public Command goToZero() {
    return run(() -> {
      SwerveModuleState[] zeroStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
      setModuleStates(zeroStates, MAX_DRIVE_SPEED);
    });
  }

  public Command rotateToAmp() {
    boolean redAlliance = shouldFlip(); // Depending on the starting alliance side, the amp would be at different angle. Account for that.
    RotateToAngle rotate = redAlliance ? new RotateToAngle(90, this) : new RotateToAngle(-90, this); // If red, go to 90. If blue, go to -90.

    return rotate;
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public Command toggleFieldOriented() {
    return runEnd(()-> {}, () -> fieldOriented = !fieldOriented);
  }

  public boolean getFieldOriented() {
    return fieldOriented;
  }

  public boolean getSlowMode() {
    return slowMode;
  }

  public Command toggleSlowMode() {
    return runEnd(() -> {}, () -> slowMode = !slowMode);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
  }

  public void drive(double xInput, double yInput, double rotateInput) {
    double xSpeed = xLimiter.calculate(xInput) * MAX_DRIVE_SPEED;
    double ySpeed = yLimiter.calculate(yInput) * MAX_DRIVE_SPEED;
    double rotateSpeed = rotateLimiter.calculate(rotateInput) * MAX_ROTATE_SPEED;

    if(fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotateSpeed, getGyro().getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed);
      }
    SwerveModuleState moduleStates[] = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates, MAX_DRIVE_SPEED);
  }
    
  public void drive(ChassisSpeeds speeds, double maxDriveSpeed) {
    chassisSpeeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    
    SwerveModuleState moduleStates[] = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates, maxDriveSpeed);
  }

  public boolean shouldFlip() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public void setBrakeMode() {
    frontLeft.setBrakeMode();
    frontRight.setBrakeMode();
    backLeft.setBrakeMode();
    backRight.setBrakeMode();
  }

  public void setCoastMode() {
    frontLeft.setCoastMode();
    frontRight.setCoastMode();
    backLeft.setCoastMode();
    backRight.setCoastMode();
  }

  public double get5V() {
    return RobotController.getVoltage5V();
  }

  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
      poseSupplier, 
      resetPoseConsumer,
      chassisSpeedSupplier,
      robotRelativeOutput,
      PATH_CONFIG,
      shouldFlipSupplier,
      this
      );
  }

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.setSmartDashboardType("Swerve drive");
    sendableBuilder.addBooleanProperty("Field Oriented", () -> fieldOriented, null);
    sendableBuilder.addBooleanProperty("Slow mode", () -> getSlowMode(), null);
    // sendableBuilder.addDoubleProperty("Back Right", ()-> backRight.getOffsets(), null);
   }  

  @Override
  public void periodic() {
    swerveOdometry.update(getRotation2d(), getModulePositions());
  }
}