// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  Spark shooter;
  /** Creates a new Shooter. */

  public Shooter() {
    shooter = new Spark(Constants.SHOOTER);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shootBall(double speed){
    shooter.set(speed);
  }

  public void stop(){
    shooter.set(0);
  }
}
