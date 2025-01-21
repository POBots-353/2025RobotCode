// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.crte.phoenix.RelativeEncoder;


public class Elevator<leftMotor, RelativeEncoder> extends SubsystemBase {
  private static final DutyCycleOut Follower = null;
    private static final String elevatorMotor = null;
        private static final String IntakeConstants = null;
      
        /** Creates a new Elevator. */

         TalonFX rightMotor = new TalonFX(1, "Cannie");
                TalonFX leftMotor = new TalonFX(2, "Cannie");
                double getSpeed;
                double getRotation;
                public Elevator() {
                  leftMotor.setControl(Follower);
                }
                
        
               private RelativeEncoder eEncoder = new RelativeEncoder();
               private TalonFX elevatorPID = new com.ctre.phoenix6.hardware.TalonFX (0); 
              TalonFX.config_kP(0,0.0);
              TalonFX.config_kD(0, 1.0);
              TalonFX.config_kI(0 0.0);
        
              public TalonFX getMotor() {
                return getMotor();
              }
              public void setMotor(TalonFX motor) {
                this.rightMotor = motor;
              }
            public void setSpeed(double speed){
            rightMotor.set(speed);
            }
            
            
            public void elevatorUp() {
              rightMotor.set(getSpeed);
            }
          public void elevatorStop() {
            rightMotor.set(0);
            leftMotor.set( 0);
          }
        
         
          public void zeroElevatorPosition() {
            ((CoreTalonFX) RelativeEncoder).setPosition(0);
          }
          
          public double get(){
            return leftMotor.get();
         }
          public void elevatorP(double elevator){}
          public void rotate(double speed){
            rightMotor.set(0);
          }
        
          
            
            private RelativeEncoder getEncoder() {
              
            }
            @Override
          public void periodic() {
            // This method will be called once per scheduler run
        }
        
          public static DutyCycleOut getFollower() {
            return Follower;
          }
        
          public TalonFX getRightMotor() {
            return rightMotor;
          }
        
          public void setRightMotor(TalonFX rightMotor) {
            this.rightMotor = rightMotor;
          }
        
          public TalonFX getLeftMotor() {
            return leftMotor;
          }
        
          public void setLeftMotor(TalonFX leftMotor) {
            this.leftMotor = leftMotor;
          }
          public static String getElevatormotor() {
            return rightMotor;
  }
  public static String getIntakeconstants() {
    return IntakeConstants;
  }
  public double getGetSpeed() {
    return getSpeed;
  }
  public void setGetSpeed(double getSpeed) {
    this.getSpeed = getSpeed;
  }
  public double getGetRotation() {
    return getRotation;
  }
  public void setGetRotation(double getRotation) {
    this.getRotation = getRotation;
  }
  public TalonFX getElevatorPIDController() {
    return getElevatorPIDController();
  }
  public void setElevatorPIDController(TalonFX elevatorPIDController) {
    this.elevatorPIDController = elevatorPIDController;
  }
  public RelativeEncoder getElevatorEncoder() {
    return elevatorEncoder;
  }
  public void setElevatorEncoder(RelativeEncoder elevatorEncoder) {
    this.elevatorEncoder = elevatorEncoder;
  }
  public TalonFX getElevatorPID() {
    return elevatorPID;
  }
  public void setElevatorPID(TalonFX elevatorPID) {
    this.elevatorPID = elevatorPID;
  }
}