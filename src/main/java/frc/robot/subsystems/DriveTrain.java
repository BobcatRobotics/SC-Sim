// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  Spark leftFront;
  Spark rightFront;
  Spark leftBack;
  Spark rightBack;
  MotorControllerGroup leftMotors;
  MotorControllerGroup rightMotors;
  DifferentialDrive drive; 
  DifferentialDrivetrainSim m_driveSim;
  DifferentialDriveOdometry m_odometry;
  //private final AnalogInput rangeFinder;
  private Field2d m_field;
  final Encoder m_leftEncoder;
  final Encoder m_rightEncoder;
  final EncoderSim m_leftEncoderSim;
  final EncoderSim m_rightEncoderSim;
  final AnalogGyro m_gyro;
  final AnalogGyroSim m_gyroSim;
  Pose2d m_pose;
 
  

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFront = new Spark(Constants.LEFT_FRONT);
    leftFront.setInverted(false);
    rightFront = new Spark(Constants.RIGHT_FRONT);
    rightFront.setInverted(false);
    leftBack = new Spark(Constants.LEFT_BACK);
    leftBack.setInverted(false);
    rightBack = new Spark(Constants.RIGHT_BACK);
    rightBack.setInverted(false);

    
    m_leftEncoder = new Encoder(0,1);
    m_rightEncoder = new Encoder(2,3);
    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    

    m_gyro = new AnalogGyro(1);
    m_gyroSim = new AnalogGyroSim(m_gyro);

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);  //Step 3
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.kWheelRadius / Constants.kEncoderResolution);
    
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    m_field= new Field2d();
    SmartDashboard.putData("Field",m_field); //Step 4
    
    leftMotors  = new  MotorControllerGroup(leftFront, leftBack);
    rightMotors = new MotorControllerGroup(rightFront, rightBack);
    drive = new DifferentialDrive(leftMotors, rightMotors);
    //rangeFinder = new AnalogInput(Constants.RANGE_FINDER);
  
   // Create the simulation model of our drivetrain. FAKE //Step 2
    m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
      KitbotGearing.k10p71,        // 10.71:1
      KitbotWheelSize.kSixInch,     // 6" diameter wheels.
      null                         // No measurement noise.

    );
    resetOdometry(Constants.INIT_POSE);
  }
  @Override
  public void simulationPeriodic() { //Step 3
    // Set the inputs to the system. Note that we need to convert
  // the [-1, 1] PWM signal to voltage by multiplying it by the
  // robot controller voltage.
  m_driveSim.setInputs(leftMotors.get() * RobotController.getInputVoltage(),
  rightMotors.get() * RobotController.getInputVoltage());

  // Advance the model by 20 ms. Note that if you are running this
  // subsystem in a separate thread or have changed the nominal timestep
  // of TimedRobot, this value needs to match it.
  m_driveSim.update(0.02);

 // Update all of our sensors.
  m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
  m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
  m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
  m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
  m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run

   m_pose=m_odometry.update(m_gyro.getRotation2d(), //Step 4
                    m_leftEncoderSim.getDistance(),
                    m_rightEncoderSim.getDistance());
  
       m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  public void driveWithJoysticks(Joystick controller, double speed)
  {
    //X axis of robot is forward, so let's call X axis of Joystick forward, which is 1
    drive.arcadeDrive(-controller.getRawAxis(Constants.JS_X_AXIS)*speed, controller.getRawAxis(Constants.JS_Y_AXIS)*speed);
    //drive.tankDrive(controller.getRawAxis(Constants.JS_X_AXIS)*speed, controller.getRawAxis(Constants.JS_X_AXIS)*speed);
    //drive.curvatureDrive(-controller.getRawAxis(Constants.JS_X_AXIS)*speed, controller.getRawAxis(Constants.JS_Y_AXIS)*speed,true);
  }

  public void driveForward(double speed){
    drive.tankDrive(1, 1);
    System.out.println("driveForward running ");
      }

  public boolean driveForTime(double setPointDistance, double speed){
      System.out.println("driveForTime called");
      driveForward(speed);
    return true;
  }

  public void stop(){
    drive.stopMotor();
  }
  //trajectory add ons

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  
   /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
/**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }
/**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //System.out.println("startTankDriveVolts");
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  
}
