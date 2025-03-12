// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AlgaeRemoverConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.util.LogUtil;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.urcl.URCL;

// import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Logged(name = "Robot")
  private final RobotContainer m_robotContainer;

  private final GcStatsCollector gcStatsCollector = new GcStatsCollector();

  public Robot() {
    double startTime = Timer.getFPGATimestamp();

    // CanBridge.runTCP();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    Epilogue.configure(
        (config) -> {
          config.root = "";
          config.backend = config.backend.lazy();
        });
    Epilogue.bind(this);

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

    Map<Integer, String> motorAliases = new HashMap<>();
    motorAliases.put(AlgaeRemoverConstants.algaeRemoverMotorID, "Algae Remover");
    motorAliases.put(ArmConstants.armMotorID, "Arm");
    motorAliases.put(IntakeConstants.groundIntakeMotorID, "Ground Intake");
    motorAliases.put(IntakeConstants.indexerMotorID, "Indexer Main");
    motorAliases.put(IntakeConstants.indexerFollowerMotorID, "Indexer Follower");
    motorAliases.put(OuttakeConstants.outtakeMotorID, "Outtake");

    URCL.start(motorAliases);

    m_robotContainer = new RobotContainer();

    addPeriodic(() -> m_robotContainer.stopIfBeamBroken(), 0.005, 0.005);

    double startupTimeSeconds = Timer.getFPGATimestamp() - startTime;
    DataLogManager.log("Startup Time (ms): " + startupTimeSeconds * 1000.0);
    PathfindingCommand.warmupCommand().withName("Pathfinding Warmup").schedule();
  }

  @Override
  public void robotPeriodic() {
    double startTime = Timer.getFPGATimestamp();

    CommandScheduler.getInstance().run();

    gcStatsCollector.update();

    // CANBusStatus canStatus = frc.robot.generated.TunerConstants.kCANBus.getStatus();
    // CANStatus rioCanStatus = RobotController.getCANStatus();

    // SmartDashboard.putNumber("CANivore/CAN Utilization %", canStatus.BusUtilization * 100.0);
    // SmartDashboard.putNumber("CANivore/Bus Off Count", canStatus.BusOffCount);
    // SmartDashboard.putNumber("CANivore/Receive Error Count", canStatus.REC);
    // SmartDashboard.putNumber("CANivore/Transmit Error Count", canStatus.TEC);
    // SmartDashboard.putNumber("CANivore/Tx Full Count", canStatus.TxFullCount);

    // SmartDashboard.putNumber(
    //     "RoboRIO/CAN Status/Utilization %", rioCanStatus.percentBusUtilization * 100.0);
    // SmartDashboard.putNumber("RoboRIO/CAN Status/Bus Off Count", rioCanStatus.busOffCount);
    // SmartDashboard.putNumber(
    //     "RoboRIO/CAN Status/Receive Error Count", rioCanStatus.receiveErrorCount);
    // SmartDashboard.putNumber(
    //     "RoboRIO/CAN Status/Transmit Error Count", rioCanStatus.transmitErrorCount);
    // SmartDashboard.putNumber("RoboRIO/CAN Status/Tx Full Count", rioCanStatus.txFullCount);

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

  private static final class GcStatsCollector {
    private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
    private final long[] lastTimes = new long[gcBeans.size()];
    private final long[] lastCounts = new long[gcBeans.size()];

    public void update() {
      long accumTime = 0;
      long accumCounts = 0;
      for (int i = 0; i < gcBeans.size(); i++) {
        long gcTime = gcBeans.get(i).getCollectionTime();
        long gcCount = gcBeans.get(i).getCollectionCount();
        accumTime += gcTime - lastTimes[i];
        accumCounts += gcCount - lastCounts[i];

        lastTimes[i] = gcTime;
        lastCounts[i] = gcCount;
      }

      SmartDashboard.putNumber("GC Stats/GC Time MS", (double) accumTime);
      SmartDashboard.putNumber("GC Stats/GC Counts", (double) accumCounts);
    }
  }
}
