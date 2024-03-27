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
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.RotateToAngle;

public class SwerveDrive extends SubsystemBase {

  private double BR_P;
  private double BR_I;
  private double BR_D;

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
      DriveConstants.PID_VALUES);

    frontRight = new SwerveModule(
      DriveConstants.FRONT_RIGHT[0], 
      DriveConstants.FRONT_RIGHT[1],
      DriveConstants.FR_ENCODER_PORT,
      false, true,
      DriveConstants.FRONT_RIGHT_OFFSET,
      DriveConstants.PID_VALUES);
   
    backLeft = new SwerveModule(
      DriveConstants.BACK_LEFT[0],
      DriveConstants.BACK_LEFT[1],
      DriveConstants.BL_ENCODER_PORT,
      false, false,
      DriveConstants.BACK_LEFT_OFFSET,
      DriveConstants.PID_VALUES);  
      
    backRight = new SwerveModule(
      DriveConstants.BACK_RIGHT[0],
      DriveConstants.BACK_RIGHT[1],
      DriveConstants.BR_ENCODER_PORT,
      false, true,
      DriveConstants.BACK_RIGHT_OFFSET,
      DriveConstants.BR_PID_VALUES);

    gyro = new AHRS(SerialPort.Port.kUSB);

    states = new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };

    swerveOdometry = new SwerveDriveOdometry(DriveConstants.KINEMATICS, new Rotation2d(gyro.getAngle()), getModulePositions());
    chassisSpeeds = new ChassisSpeeds();

    poseSupplier = () -> getPose();
    resetPoseConsumer = pose -> resetOdometry(pose);
    robotRelativeOutput = inputSpeed -> drive(inputSpeed, DriveConstants.SLOWER_DRIVE_SPEED);
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
      setModuleStates(zeroStates, DriveConstants.MAX_DRIVE_SPEED);
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
    
  public void drive(ChassisSpeeds speeds, double maxDriveSpeed) {
    chassisSpeeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    
    SwerveModuleState moduleStates[] = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates, maxDriveSpeed);
  }

<<<<<<< HEAD
=======
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

>>>>>>> 187e1594b3aa41e02778501c32034abe4a158cc9
  public double get5V() {
    return RobotController.getVoltage5V();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
  }

  public void drive(double xInput, double yInput, double rotateInput) {
    SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    SlewRateLimiter rotateLimiter = new SlewRateLimiter(3);
    
    double xSpeed = xLimiter.calculate(xInput) * DriveConstants.MAX_DRIVE_SPEED;
    double ySpeed = yLimiter.calculate(yInput) * DriveConstants.MAX_DRIVE_SPEED;
    double rotateSpeed = rotateLimiter.calculate(rotateInput) * DriveConstants.MAX_ROTATE_SPEED;

    if(fieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotateSpeed, getGyro().getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed);
      }
    }

    
  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState moduleStates[] = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates, DriveConstants.MAX_DRIVE_SPEED);
  }

  public boolean shouldFlip() {
    var alliance = DriverStation.getAlliance();
    if (DriverStation.getAlliance().isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry,
      this::getChassisSpeeds,
      this::drive,
      DriveConstants.PATH_CONFIG,
      this::shouldFlip,
      this
<<<<<<< HEAD
      ); 
=======
      );
>>>>>>> 187e1594b3aa41e02778501c32034abe4a158cc9
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