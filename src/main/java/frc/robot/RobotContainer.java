// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
//import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
//import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.simulation.JoystickSim;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.AutoShoot;
//import frc.robot.commands.AutonomousOne;
//import frc.robot.commands.AutonomousTwo;
//import frc.robot.commands.DriveForTime;
//import frc.robot.commands.DriveWithJoysticks;

//SAVE
//import frc.robot.commands.RamseteAuto;

//import frc.robot.commands.ShootBall;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
//import edu.wpi.first.wpilibj2.command.RamseteCommand;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
   // private final DriveWithJoysticks driveWithJoystick;
    //private final DriveForTime driveForTime;
    //private final DriveForTime driveToDistance;

    //public static Joystick driverJoystick;
     
    //Shooter declare
    //private final Shooter shooter;
    //private final ShootBall shootBall;
    //private final AutoShoot autoShoot;

    //private final AutonomousOne autonomousOne;
    //private final AutonomousTwo autonomousTwo;

    //SAVE
    //private final RamseteAuto ramseteAuto;


    //SendableChooser<Command> chooser = new SendableChooser<>();

   

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    //driveWithJoystick = new DriveWithJoysticks(driveTrain);  //?? so this is an attribute? But is is an object with methods?
   // driveWithJoystick.addRequirements(driveTrain);
   // driveTrain.setDefaultCommand(driveWithJoystick);

    //driveForTime = new DriveForTime(driveTrain);
    //driveForTime.addRequirements(driveTrain);

    //driveToDistance = new DriveForTime(driveTrain);  
    //driveToDistance.addRequirements(driveTrain);

    
    //driverJoystick = new XboxController(Constants.JOYSTICK_NUMBER);
    //driverJoystick = new Joystick(0);

    //shooter = new Shooter();
    //shootBall = new ShootBall(shooter);
    //shootBall.addRequirements(shooter);

   // autoShoot = new AutoShoot(shooter);
   // autoShoot.addRequirements(shooter);

    //autonomousOne = new AutonomousOne(driveTrain, shooter);
    //autonomousTwo = new AutonomousTwo(driveTrain);
    

    //SAVE
    //ramseteAuto = new RamseteAuto(driveTrain);

    //Add choices  as options here
    //chooser.addOption("Autonomous One",autonomousOne);
    //chooser.addOption("Autonomous Two",autonomousTwo);
   // chooser.addOption("RamseteAuto",ramseteAuto);
    //Default option
    //chooser.setDefaultOption("Default", autonomousOne);
    //Add to Choices to SmartDashboard
    //SmartDashboard.putData("Autotnomous", chooser);

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
    //JoystickButton shootButton = new JoystickButton(driverJoystick, 1);
    //shootButton.whileHeld(new ShootBall(shooter));

    //JoystickButton aButton = new JoystickButton(driverJoystick, 2);
    //aButton.whenPressed(new DriveForTime(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return ramseteAuto;
  
    //return chooser.getSelected(); //ignore for trying autonomous trajectory
     // Create a voltage constraint to ensure we don't accelerate too fast
      /*var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
              Constants.ksVolts,
              Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),
              Constants.kDriveKinematics,
          10);
*/
    // Create config for trajectory

  /* TrajectoryConfig config =
      new TrajectoryConfig(
              Constants.kMaxSpeedMetersPerSecond,
              Constants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);
*/
     String trajectoryJSON = "/Users/stevencohen/Documents/RobotCode22/SCSim/PathWeaver/output/Ball3and4.wpilib.json";
    Trajectory pickUpBall2Trajectory = new Trajectory();

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    try {
      pickUpBall2Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }
    // An example trajectory to follow.  All units in meters.
    /*
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          Constants.INIT_POSE,
          //new Pose2d(5, 5, new Rotation2d(270)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(
              new Translation2d(2, 3), 
              new Translation2d(4, 5)
          ),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(5, 4, new Rotation2d(0)),
          // Pass config
          config
      );
    */

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
