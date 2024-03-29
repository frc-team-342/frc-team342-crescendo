// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class DriveDistance extends Command {
  /** Creates a new DriveDistance. */
  private final double distance;
  private final double velocity;
  private final SwerveDrive drive;
  private final PIDController rotationController;

  private Pose2d startingPose;
  private Pose2d endingPose;


  public DriveDistance(double distance, double velocityInput, SwerveDrive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = distance;
   
    this.velocity = (distance <= 0)
    ? MathUtil.clamp(Math.abs(velocityInput), 0.0, DriveConstants.MAX_DRIVE_SPEED)
    : -Math.abs(MathUtil.clamp(velocityInput, -DriveConstants.MAX_DRIVE_SPEED, DriveConstants.MAX_DRIVE_SPEED));

    this.drive = drive;

    rotationController = new PIDController(
      DriveConstants.DRIVE_DISTANCE_ROTATE_CONTROLLER_P, 
      DriveConstants.DRIVE_DISTANCE_ROTATE_CONTROLLER_I,
      DriveConstants.DRIVE_DISTANCE_ROTATE_CONTROLLER_D);

    addRequirements(drive);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPose = drive.getPose();

      endingPose = startingPose.transformBy(new Transform2d(
        new Translation2d(distance, startingPose.getRotation()),
        new Rotation2d()
      ));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Rotation2d current = drive.getRotation2d();
    Rotation2d error = current.minus(startingPose.getRotation());

    double rotation = rotationController.calculate(error.getRadians(), 0);

    //X and Y are swapped to account for robot control system behavior
    var speeds = new ChassisSpeeds(velocity + rotation, 0, 0);

    drive.drive(speeds, DriveConstants.MAX_DRIVE_SPEED); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds(), distance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //Checks the robot's current position
    Pose2d current = drive.getPose();

    //Calculates the difference in distance from the starting and ending positions
    double difference = current.getTranslation().getDistance(startingPose.getTranslation());

    //Checks to make sure the distance traveled is within tolerance
    double absoluteDistance = Math.abs(distance);
    return difference > (absoluteDistance - DriveConstants.DISTANCE_TOLERANCE) && difference < (absoluteDistance + DriveConstants.DISTANCE_TOLERANCE);
  }

  @Override
  public void initSendable(SendableBuilder builder){
  builder.addStringProperty("Current Position",
    () -> {
              Pose2d current = drive.getPose();
              String currentPosition = String.format("(%f, %f)", current.getX(), current.getY());
              return currentPosition;
          }, null);

          builder.addStringProperty("End Position",
            () -> {
                    String endPosition = String.format("(f%, f%)", endingPose.getX(), endingPose.getY());
                    return endPosition;
                  }, null);
  }
}
