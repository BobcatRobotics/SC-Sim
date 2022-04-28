// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveForTime extends CommandBase {
  DriveTrain driveTrain;
  private final Timer m_timer = new Timer();
 
  private boolean finish = false;
  /** Creates a new DriveToDistance. */
  public DriveForTime(DriveTrain dt) {
    driveTrain = dt;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize DriveForTime");
    m_timer.reset();
    m_timer.start();
    finish = false;
  }
   
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if (m_timer.get() < 0.1) {
      driveTrain.driveForward(1.0);
      System.out.println("m_timer="+m_timer.get());
    } else {
  finish = true;
    }
}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finish DriveForTime");
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //checking to see if finished, returns true or false
    return finish;
  }
}
