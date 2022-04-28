// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    ///PWM locaations
    public static final int LEFT_FRONT = 0;
    public static final int RIGHT_FRONT = 1;
    public static final int LEFT_BACK = 2;
    public static final int RIGHT_BACK = 3;
    public static final int SHOOTER = 4;
    public static final int INTAKE = 5;

    //analog inputs
    public static final int RANGE_FINDER = 0;

    //Axis
    public static final int JS_X_AXIS = 1;//since X axis of robot is forward
    public static final int JS_Y_AXIS = 0;//with arcade drive, this is Z rotation
    public static final int RIGHT_TRIGGER = 3;
    public static final int JS_THETA_AXIS = 2;

    public static final Pose2d INIT_POSE = new Pose2d(1, 1, Rotation2d.fromDegrees(0));


    public static final double DRIVETRAINSPEED = 0.7;
    public static final double DRIVE_FORWARD_TIME = 3.0;
    public static final double AUTONOMOUS_SPEED = 0.4;
    public static final int JOYSTICK_NUMBER = 0;
    public static final double SHOOTER_SPEED = 0.5;
    public static final double kWheelRadius = 3.0;
    public static final double kEncoderResolution = 2048;
    public static final double INTAKE_SPEED = 0.7;
    public static final double AUTO_SHOOT_TIME = 2.0;
    public static final double SETPOINT_FORWARD = 1.5;


    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    private static final double kTrackwidthMeters = .3;
    public static final DifferentialDriveKinematics kDriveKinematics =  new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kPDriveVel = 8.5;
    public static final double ksVolts=.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
   
    
  
   
    
}
