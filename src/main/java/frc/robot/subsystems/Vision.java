// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Utilities;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
    public Vision() {}

    public Pose2d getPoseEstimate(String cameraID) {
        Pose2d redEstimate = LimelightHelpers.getBotPose2d_wpiRed(cameraID);
        Pose2d blueEstimate = LimelightHelpers.getBotPose2d_wpiBlue(cameraID);

        return Utilities.getAlliance() == Alliance.Red ? redEstimate : blueEstimate;
    }

    public Pose2d[] getPoseEstimates() {
        return new Pose2d[] {
            getPoseEstimate(VisionConstants.leftCamID),
            getPoseEstimate(VisionConstants.rightCamID)
        };
    }

    @Override
    public void periodic() {}
}
