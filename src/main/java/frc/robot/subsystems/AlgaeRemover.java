package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.Constants.PreMatchConstants;
import frc.robot.util.ExpandedSubsystem;

// TODO:
// Increase Speed of Algae Remover
// Automate it as much as possible.

@Logged(strategy = Strategy.OPT_IN)
public class AlgaeRemover extends ExpandedSubsystem {
  private SparkMax algaeRemoverMotor;

  private SparkClosedLoopController algaeRemoverPIDController;
  private SparkAbsoluteEncoder algaeRemoverAbsoluteEncoder;
  private RelativeEncoder algaeRemoverEncoder;

  private double positionTolerance = Units.degreesToRadians(3.53);

  public AlgaeRemover() {
    algaeRemoverMotor =
        new SparkMax(AlgaeRemoverConstants.algaeRemoverMotorID, MotorType.kBrushless);
    algaeRemoverAbsoluteEncoder = algaeRemoverMotor.getAbsoluteEncoder();
    algaeRemoverEncoder = algaeRemoverMotor.getEncoder();
    algaeRemoverPIDController = algaeRemoverMotor.getClosedLoopController();
    SparkMaxConfig algaeRemoverConfig = new SparkMaxConfig();

    algaeRemoverConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(AlgaeRemoverConstants.absoluteEncoderConversion)
        .zeroCentered(true);

    algaeRemoverConfig
        .encoder
        .positionConversionFactor(AlgaeRemoverConstants.internalEncoderConversion)
        .velocityConversionFactor(AlgaeRemoverConstants.internalEncoderConversion);
    // .zeroOffset(.113922);

    algaeRemoverConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    algaeRemoverConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeRemoverConstants.currentLimit)
        .secondaryCurrentLimit(AlgaeRemoverConstants.shutOffCurrentLimit);

    algaeRemoverConfig.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot0).p(2.8).i(0.0).d(0.353);

    // algaeRemoverConfig
    //     .closedLoop
    //     .maxMotion
    //     .maxVelocity(AlgaeRemoverConstants.maxVelocity)
    //     .maxAcceleration(AlgaeRemoverConstants.maxAcceleration);

    // algaeRemoverConfig
    //     .softLimit
    //     .forwardSoftLimit(AlgaeRemoverConstants.minPosition.in(Degrees))
    //     .forwardSoftLimitEnabled(true)
    //     .reverseSoftLimit(AlgaeRemoverConstants.maxPosition.in(Degrees))
    //     .reverseSoftLimitEnabled(true);

    algaeRemoverMotor.configure(
        algaeRemoverConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeRemoverEncoder.setPosition(getAbsolutePosition().in(Radians));
  }

  public Command moveToPosition(Angle targetAngle) {
    return run(() ->
            algaeRemoverPIDController.setReference(
                targetAngle.in(Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0))
        .until(() -> atSetpoint(targetAngle))
        .withName("Algae Remover move to " + targetAngle.in(Degrees) + " degrees");
  }

  public boolean atSetpoint(Angle targetAngle) {
    return Math.abs(targetAngle.in(Radians) - algaeRemoverEncoder.getPosition())
        < positionTolerance;
  }

  public void stopAlgaeRemover() {
    algaeRemoverMotor.set(0);
  }

  public void algaeRemoverUp() {
    algaeRemoverMotor.set(AlgaeRemoverConstants.algaeRemoverSpeed);
  }

  public void algaeRemoverDown() {
    algaeRemoverMotor.set(-AlgaeRemoverConstants.algaeRemoverSpeed);
  }

  @Logged(name = "Absolute Position")
  public Angle getAbsolutePosition() {
    return Radians.of(algaeRemoverAbsoluteEncoder.getPosition());
  }

  @Logged(name = "Position")
  public Angle getPosition() {
    return Radians.of(algaeRemoverAbsoluteEncoder.getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae Remover/Position", getPosition().in(Degrees));
  }

  @Override
  public Command getPrematchCheckCommand() {
    return Commands.sequence(
        // Check for hardware errors
        Commands.runOnce(
            () -> {
              REVLibError error = algaeRemoverMotor.getLastError();
              if (error != REVLibError.kOk) {
                addError("Intake motor error: " + error.name());
              } else {
                addInfo("Intake motor contains no errors");
              }
            }),
        // Checks Ground Intake Motor
        Commands.parallel(
            moveToPosition(AlgaeRemoverConstants.intakePosition),
            Commands.sequence(
                Commands.waitSeconds(PreMatchConstants.prematchDelay),
                Commands.runOnce(
                    () -> {
                      addInfo("Algae Remover Motor is moving");
                      if (Math.abs(
                              AlgaeRemoverConstants.intakePosition.minus(getPosition()).in(Degrees))
                          > 10) {
                        addError("Algae Remover Motor is not at desired position");
                        // We just put a fake range for now; we'll update this later on
                      } else {
                        addInfo("Algae Remover Motor is at the desired position");
                      }
                    }))),
        moveToPosition(AlgaeRemoverConstants.bargePosition),
        Commands.waitSeconds(PreMatchConstants.prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(AlgaeRemoverConstants.bargePosition.minus(getPosition()).in(Degrees))
                  > 10) {
                addError("Algae Remover Motor isn't fully down");
              } else {
                addInfo("Algae Rmeover motor is fully down");
              }
            }));
  }
}
