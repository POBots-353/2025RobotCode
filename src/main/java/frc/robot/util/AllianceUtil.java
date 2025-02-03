package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.util.Optional;
import java.util.Set;

public class AllianceUtil {
  public static final Set<Integer> RED_REEF_IDS = Set.of(6, 7, 8, 9, 10, 11);
  public static final Set<Integer> BLUE_REEF_IDS = Set.of(17, 18, 19, 20, 21, 22);

  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public static Set<Integer> getReefIds() {
    return isRedAlliance() ? RED_REEF_IDS : BLUE_REEF_IDS;
  }

  public static Pose2d getReefPose() {
    return isRedAlliance() ? FieldConstants.reefRedAlliance : FieldConstants.reefBlueAlliance;
  }
}
