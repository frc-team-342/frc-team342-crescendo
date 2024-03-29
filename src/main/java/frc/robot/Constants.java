// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

public static class IntakeConstants {
  public static final int INTAKE_SENSOR = 1;
  
  public static final int INTAKE_MOTOR = 10;

  public static final int WRIST_ID = 9;
  public static final int LEFT_ELEV_ID = 13;
  public static final int RIGHT_ElEV_ID = 14;

  public static final int MAG_ENCODER = 7;

  public static final double INTAKE_SPEED = 0.6;
  public static final double INTAKE_SHOOT_SPEED = 1;
  public static final double FEED_SHOOTER_SPEED = 1; 
  public static final double WRIST_SPEED = 0.7;

public static final double LOW_WRIST_POS = 0.9;
  public static final double HIGH_WRIST_POS = 0.311;
  public static final double AMP_POS = 0.531; // In memoriam of 0.342 :(

  public static final double MAX_DISTANCE = 83;
  
  public static final double DESIRED_SPEED = 5100;
  
  public static final double DEFAULT_CURRENT = 30;
}

  public static class DriveConstants {

    public static final double TRACK_WIDTH = Units.inchesToMeters(29); // *

    public static final double DRIVE_GEAR_RATIO = 1 / 6.75;
    public static final double ROTATE_GEAR_RATIO = 1 / 12.75;

    public static final double MAX_DRIVE_SPEED = Units.feetToMeters(15.1); // M/S
    public static final double SLOWER_DRIVE_SPEED = Units.feetToMeters(5);

    public static final double MAX_ROTATE_SPEED = 4 * Math.PI; // M/S
    public static final double MAX_ACCELERATION = Units.feetToMeters(5);
    public static final double MAX_RPS = 5820;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI;

    public static final double DRIVE_POSITION_CONVERSION = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60;

    public static final double ROTATE_POSITION_CONVERSION = ROTATE_GEAR_RATIO * Math.PI * 2;
    public static final double ROTATE_VELOCITY_CONVERSION = ROTATE_POSITION_CONVERSION / 60;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2),
      new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2),
      new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2),
      new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2)
    );

    public static final double[] FL_PID_VALUES = {0.5, 0, 0};
    public static final double[] FR_PID_VALUES = {0.3, 0, 0};
    public static final double[] BL_PID_VALUES = {0.5, 0.01, 0};
    public static final double[] BR_PID_VALUES = {0.3, 0, 0};

    // Drive Motor IDs
    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_ID = 2;
    public static final int BACK_LEFT_DRIVE_ID = 3;  
    public static final int BACK_RIGHT_DRIVE_ID = 4;

    // Rotate Motor IDs
    public static final int FRONT_LEFT_ROTATE_ID = 5;
    public static final int FRONT_RIGHT_ROTATE_ID = 6;
    public static final int BACK_LEFT_ROTATE_ID = 7;
    public static final int BACK_RIGHT_ROTATE_ID = 8;

    // Encoder Ports
    public static final int FL_ENCODER_PORT = 1;
    public static final int FR_ENCODER_PORT = 0;
    public static final int BL_ENCODER_PORT = 3;
    public static final int BR_ENCODER_PORT = 2;

    // Swerve Modules
    public static final int[] FRONT_LEFT = {FRONT_LEFT_DRIVE_ID, FRONT_LEFT_ROTATE_ID};
    public static final int[] FRONT_RIGHT = {FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_ROTATE_ID};
    public static final int[] BACK_LEFT = {BACK_LEFT_DRIVE_ID, BACK_LEFT_ROTATE_ID};
    public static final int[] BACK_RIGHT = {BACK_RIGHT_DRIVE_ID, BACK_RIGHT_ROTATE_ID};

    // Offsets
    public static final double FRONT_LEFT_OFFSET = 4.94;
    public static final double FRONT_RIGHT_OFFSET = 5.47;
    public static final double BACK_LEFT_OFFSET = 5.47;
    public static final double BACK_RIGHT_OFFSET = 4.22;

    public static final HolonomicPathFollowerConfig PATH_CONFIG = new HolonomicPathFollowerConfig(
      new PIDConstants(5, 0, 0), 
      new PIDConstants(5, 0, 0), 
      SLOWER_DRIVE_SPEED, 
      (TRACK_WIDTH / 2), 
      new ReplanningConfig()
      );

      //PID Constants for the DriveDistance Rotation PID Controller
      public static final double DRIVE_DISTANCE_ROTATE_CONTROLLER_P = 0.3;
      public static final double DRIVE_DISTANCE_ROTATE_CONTROLLER_I = 0.0;
      public static final double DRIVE_DISTANCE_ROTATE_CONTROLLER_D = 0.0;

      //Tolerance for the DriveDistance Command
      public static final double DISTANCE_TOLERANCE = Units.metersToFeet(0.3);

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class OuttakeConstants {

    public static final int MOTOR_ONE_ID = 12;
    public static final int MOTOR_TWO_ID = 11;

    public static final double P_VALUE = 0.0001;
    public static final double I_VALUE = 0;
    public static final double D_VALUE = 0;
public static final double FF_VALUE = 0.5;
    
    public static final int CURRENT_LIMIT = 60;
  }

  public static class LimelightConstants{

    //Amp Side Limelight height constants
    public static final double AMP_SIDE_LIMELIGHT_HEIGHT_TO_SPEAKER = 0;
    public static final double AMP_SIDE_LIMELIGHT_HEIGHT_TO_AMP = 0;
    public static final double AMP_SIDE_LIMELIGHT_HEIGHT_TO_SOURCE = 0;

    //Shooter Side Limelight height constants
    public static final double SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_SPEAKER = Units.inchesToMeters(38.225);
    public static final double SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_AMP = 0;
    public static final double SHOOTER_SIDE_LIMELIGHT_HEIGHT_TO_SOURCE = 0;

    //Amp Side Apriltag offset constants
    public static final double AMP_SIDE_APRILTAG_DRIVE_OFFSET = 0;
    public static final double AMP_SIDE_APRILTAG_STRAFE_OFFSET = 0;
    public static final double AMP_SIDE_APRILTAG_ROTATE_OFFSET = 0;

    //Shooter Side Apriltag offset constants
    public static final double SHOOTER_SIDE_APRILTAG_DRIVE_OFFSET = 0;
    public static final double SHOOTER_SIDE_APRILTAG_STRAFE_OFFSET = 0;
    public static final double SHOOTER_SIDE_APRILTAG_ROTATE_OFFSET = 0;

    //Ready-to-Shoot offset constants
    public static final double MINIMUM_DISTANCE_FROM_SUBWOOFER = Units.inchesToMeters(33.25);
    public static final double MAXIMUM_DISTANCE_FROM_SUBWOOFER = Units.inchesToMeters(44);
    public static final double MINIMUM_DISTANCE_FROM_SPEAKER = Units.inchesToMeters(37.7) + MINIMUM_DISTANCE_FROM_SUBWOOFER; //Adds distance from subwoofer to apriltag 8 and minimum distance from subwoofer
    public static final double MAXIMUM_DISTANCE_FROM_SPEAKER = Units.inchesToMeters(37.7) + MAXIMUM_DISTANCE_FROM_SUBWOOFER; //Adds distance from subwoofer to apriltag 8 and maximum distance from subwoofer
    public static final double MINIMUM_ANGLE_OFFSET_FROM_SPEAKER = -27;
    public static final double MAXIMUM_ANGLE_OFFSET_FROM_SPEAKER = 27;
    
    //Limelight name constants
    public static final String AMP_SIDE_LIMELIGHT_NAME = "limelight-amp";
    public static final String SHOOTER_SIDE_LIMELIGHT_NAME = "limelight-shooter";
  }

}
