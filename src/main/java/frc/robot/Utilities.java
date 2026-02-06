// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utilities {
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().get();
    }

    public static boolean isValidPose(Pose2d pose) {
        return !pose.equals(new Pose2d()) && pose != null;
    }
}
