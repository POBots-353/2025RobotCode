// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;



public class ES<leftMotor, RelativeEncoder> extends SubsystemBase {
  private static final DutyCycleOut Follower = null;
  /** Creates a new Elevator. */

  TalonFX rightMotor = new TalonFX(1, "Cannie");
  AutoConstants PIDConstants = new AutoConstants(); 
  TalonFX leftMotor = new TalonFX(2, "Cannie");
  double getSpeed;
  double getRotation;

  public ES() {
    leftMotor.setControl(Follower);
  }
  
  
  public void setMotor(TalonFX motor) {
    this.rightMotor = motor;
    this.leftMotor = motor;
  }
  
  public void setSpeed(double speed){
    rightMotor.set(1);
    rightMotor.set(1);
  }  

  public double getSpeed(double speed){
    return rightMotor.get();

  }                
            
  public void elevatorUp(int speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);

  }
              
  public void elevatorDown(int speed) {
    rightMotor.set(-speed);
    leftMotor.set(-speed);

  }

  public void elevatorStop() {
    rightMotor.set(0);
    leftMotor.set( 0);
  }

  // public void rotate(double speed){
  //   rightMotor.set(5);
  // }
        
}          
            
    

        //    @Override
        //  public void periodic() {}
            // This method will be called once per scheduler run
        
    
      
    