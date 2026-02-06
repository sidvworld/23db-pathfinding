// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Rotation2d redPerspective = Rotation2d.kZero, bluePerspective = Rotation2d.k180deg;
    
    public static final Pose2d hubPose = new Pose2d(4.625, 4.03, new Rotation2d());
}
