// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LogUtil;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    double startTime = Timer.getFPGATimestamp();

    CanBridge.runTCP();

    DataLogManager.start();

    DriverStation.startDataLog(DataLogManager.getLog());

    SignalLogger.start();

    LogUtil.recordMetadata("Java Vendor", System.getProperty("java.vendor"));
    LogUtil.recordMetadata("Java Version", System.getProperty("java.version"));
    LogUtil.recordMetadata("WPILib Version", WPILibVersion.Version);

    LogUtil.recordMetadata(
        "REVLib Version",
        SparkBase.kAPIMajorVersion
            + "."
            + SparkBase.kAPIMinorVersion
            + "."
            + SparkBase.kAPIBuildVersion);
    LogUtil.recordMetadata("Runtime Type", getRuntimeType().toString());

    // Git and build information
    LogUtil.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
    LogUtil.recordMetadata("Build Date", BuildConstants.BUILD_DATE);
    LogUtil.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    LogUtil.recordMetadata("Git Date", BuildConstants.GIT_DATE);
    LogUtil.recordMetadata("Git Revision", BuildConstants.GIT_REVISION);
    LogUtil.recordMetadata("Git Branch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        LogUtil.recordMetadata("Git Dirty", "All changes committed");
        break;
      case 1:
        LogUtil.recordMetadata("Git Dirty", "Uncommitted changes");
        break;
      default:
        LogUtil.recordMetadata("Git Dirty", "Unknown");
        break;
    }

    m_robotContainer = new RobotContainer();

    double startupTimeSeconds = Timer.getFPGATimestamp() - startTime;
    DataLogManager.log("Startup Time (ms): " + startupTimeSeconds * 1000.0);
    // PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    double startTime = Timer.getFPGATimestamp();

    CommandScheduler.getInstance().run();

    CANBusStatus canStatus = TunerConstants.kCANBus.getStatus();
    CANStatus rioCanStatus = RobotController.getCANStatus();

    SmartDashboard.putNumber("CANivore/CAN Utilization %", canStatus.BusUtilization * 100.0);
    SmartDashboard.putNumber("CANivore/Bus Off Count", canStatus.BusOffCount);
    SmartDashboard.putNumber("CANivore/Receive Error Count", canStatus.REC);
    SmartDashboard.putNumber("CANivore/Transmit Error Count", canStatus.TEC);
    SmartDashboard.putNumber("CANivore/Tx Full Count", canStatus.TxFullCount);

    SmartDashboard.putNumber(
        "RoboRIO/CAN Status/Utilization %", rioCanStatus.percentBusUtilization * 100.0);
    SmartDashboard.putNumber("RoboRIO/CAN Status/Bus Off Count", rioCanStatus.busOffCount);
    SmartDashboard.putNumber(
        "RoboRIO/CAN Status/Receive Error Count", rioCanStatus.receiveErrorCount);
    SmartDashboard.putNumber(
        "RoboRIO/CAN Status/Transmit Error Count", rioCanStatus.transmitErrorCount);
    SmartDashboard.putNumber("RoboRIO/CAN Status/Tx Full Count", rioCanStatus.txFullCount);

    SmartDashboard.putNumber("RoboRIO/CPU Temperature", RobotController.getCPUTemp());
    SmartDashboard.putBoolean("RoboRIO/RSL", RobotController.getRSLState());
    SmartDashboard.putNumber("RoboRIO/Input Current", RobotController.getInputCurrent());

    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    double codeRuntime = (Timer.getFPGATimestamp() - startTime) * 1000.0;
    SmartDashboard.putNumber("Code Runtime (ms)", codeRuntime);
  }

  @Override
  public void disabledInit() {
    System.gc();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
