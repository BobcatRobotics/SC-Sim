// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class RamseteAuto extends CommandBase {
  /** Creates a new RamseteAuto. 
 * @param dt
   * @param dt*/
    DriveTrain driveTrain;
    boolean finish;

  public RamseteAuto(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
   driveTrain = dt;
  addRequirements(driveTrain);}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

     finish = false;
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
      10);

    // Create config for trajectory

  TrajectoryConfig config =
    new TrajectoryConfig(
      Constants.kMaxSpeedMetersPerSecond,
      Constants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(Constants.kDriveKinematics)// Apply the voltage constraint
      .addConstraint(autoVoltageConstraint
    );


  // An example trajectory to follow.  All units in meters.
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
  System.out.println("ready to instantiate ramseteCommand");
  final DifferentialDriveKinematics m_kinematics;
  m_kinematics = Constants.kDriveKinematics;
  RamseteCommand ramseteCommand =
    new RamseteCommand(
      exampleTrajectory,
      driveTrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter),
      m_kinematics,
      driveTrain::getWheelSpeeds,
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      driveTrain::tankDriveVolts,
      driveTrain
    );

  Pose2d poseTemp = driveTrain.getPose();
  System.out.println("posetemp "+poseTemp);

  // Reset odometry to the starting pose of the trajectory.
  driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

  // Run path following command, then stop at the end.

   /*added stuff for debugging
     final Trajectory.State initialState;
      DifferentialDriveWheelSpeeds m_prevSpeeds;
      initialState = exampleTrajectory.sample(0);
    m_prevSpeeds = m_kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
  System.out.println("m_prevSpeeds.left "+m_prevSpeeds.leftMetersPerSecond);
    //return 
    */
   ramseteCommand.execute();
  driveTrain.tankDriveVolts(0, 0);
 
    //
  
}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
          // Create a voltage constraint to ensure we don't accelerate too fast
   return finish;
  } 
}
