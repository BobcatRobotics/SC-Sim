// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;

//SAVE
//import frc.robot.commands.RamseteAuto;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 
    //Drivetrain declare
    private  final DriveTrain driveTrain;
   
    //SAVE
    //private final RamseteAuto ramseteAuto;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    
    //SAVE
    //ramseteAuto = new RamseteAuto(driveTrain);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    String trajectoryJSON = "/Users/stevencohen/Documents/RobotCode22/SCSim/PathWeaver/output/Ball3and4.wpilib.json";
    Trajectory pickUpBall2Trajectory = new Trajectory();

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    try {
      pickUpBall2Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }
   
    Trajectory exampleTrajectory = pickUpBall2Trajectory;

    RamseteCommand ramseteCommand =
      new RamseteCommand(
          exampleTrajectory,
          driveTrain::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(
              Constants.ksVolts,
              Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          driveTrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          driveTrain::tankDriveVolts,
          driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

   // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }
}
