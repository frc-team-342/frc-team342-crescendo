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
  public static class DriveConstants { // ***UPDATE CONSTANTS***

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

    public static final double[] PID_VALUES = {0.5, 0, 0};
    public static final double[] BL_PID_VALUES = {0.3, 0, 0.05};

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
    public static final double FRONT_LEFT_OFFSET = 1.88;
    public static final double FRONT_RIGHT_OFFSET = 2.35;
    public static final double BACK_LEFT_OFFSET = 3.39;
    public static final double BACK_RIGHT_OFFSET = 1.12;

    public static final HolonomicPathFollowerConfig PATH_CONFIG = new HolonomicPathFollowerConfig(
      new PIDConstants(5, 0, 0), 
      new PIDConstants(5, 0, 0), 
      SLOWER_DRIVE_SPEED, 
      (TRACK_WIDTH / 2), 
      new ReplanningConfig()
      );
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
