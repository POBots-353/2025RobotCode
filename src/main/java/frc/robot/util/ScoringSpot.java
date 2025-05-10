// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ScoringSpot {

    public final Pose2d position;
    public boolean isAvailable;
    public final int level;

     public static final List<ScoringSpot> allSpots = new ArrayList<>();

    static {
        allSpots.add(new ScoringSpot(new Pose2d(3.0, 5.0, Rotation2d.kZero), true, 3));
        allSpots.add(new ScoringSpot(new Pose2d(2.0, 4.0, Rotation2d.kZero), true, 2));
    }


    public ScoringSpot(Pose2d position, boolean isAvailable, int level) {
        this.position = position;
        this.isAvailable = isAvailable;
        this.level = level;
    }
    public static ScoringSpot getBestScoringSpot(Pose2d currentPose) {
        double bestScore = Double.NEGATIVE_INFINITY;
        ScoringSpot bestSpot = null;
    
        for (ScoringSpot spot : allSpots) {
            if (!spot.isAvailable) continue;
    
            double distance = currentPose.getTranslation().getDistance(spot.position.getTranslation());
    
            double score = (spot.level * 10.0) - (distance * 5.0); //temp
    
            if (score > bestScore) {
                bestScore = score;
                bestSpot = spot;
            }
        }
    
        return bestSpot;
    }

    public static void markSpotAsScored(ScoringSpot spot)  {
        spot.isAvailable = false;
    }

    public static void markSpotAsEmpty(ScoringSpot spot)  {
        spot.isAvailable = true;
    }

    public static Pose2d getPosition(ScoringSpot spot) {
        return spot.position;
    }

    public static boolean isAvailable(ScoringSpot spot) {
        return spot.isAvailable;
    }

    public static int getLevel(ScoringSpot spot) {
        return spot.level;
    }

    public static void resetAllSpots() {
        for (ScoringSpot spot : allSpots) {
            spot.isAvailable = true;
        }
    }
    
}
