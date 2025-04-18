// package frc.robot.subsystems;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.epilogue.Logged.Strategy;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.util.ExpandedSubsystem;

// @Logged(strategy = Strategy.OPT_IN)
// public class Climber extends ExpandedSubsystem {
//   private TalonFX climberMotor;
//   private StatusSignal<Angle> climberPosition;

//   public Climber() {

//     climberMotor = new TalonFX(ClimberConstants.climberMotorID);
//     climberMotor.getConfigurator().apply(ClimberConstants.climberConfigs);
//     climberMotor.setPosition(0);
//     climberPosition = climberMotor.getPosition();
//   }

//   public Command startWinch() {
//     return run(() -> climberMotor.set(ClimberConstants.winchSpeed));
//   }

//   public Command stopWinch() {
//     return run(() -> climberMotor.set(0));
//   }

//   public void stop() {
//     climberMotor.set(0);
//   }

//   public Command zeroPosition() {
//     return runOnce(() -> climberMotor.setPosition(0));
//   }

//   public Command moveToSetup() {
//     return run(() -> climberMotor.set(ClimberConstants.winchSpeed))
//         .until(() -> atSetpoint())
//         .unless(() -> atSetpoint())
//         .finallyDo(this::stop);
//   }

//   public boolean atSetpoint() {
//     return Math.abs(ClimberConstants.setupPosition - climberPosition.getValueAsDouble())
//         < ClimberConstants.positionTolerance;
//   }

//   @Override
//   public void periodic() {
//     climberPosition.refresh();
//     SmartDashboard.putNumber("Climber Encoder (Rotations)", climberPosition.getValueAsDouble());
//     SmartDashboard.putBoolean("Climber atSetpoint", atSetpoint());
//   }
// }
