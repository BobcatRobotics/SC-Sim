// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  Spark intake; //declare a class attribute, intake[a motor controller]of type Spark, later to have a value assigned of a specific PWM channel
  /** Creates a new Intake. */
  public Intake() { //constructor method
    intake = new Spark(Constants.INTAKE); //set the initial value for class attribute intake -could be a constant, but we are instantiating a new object of class Spark
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void intakeBall(Joystick controller, double speed){
    intake.set(controller.getRawAxis(Constants.RIGHT_TRIGGER)*speed);
  }
public void stop(){
    intake.set(0);
  }
}
