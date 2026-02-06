// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private CommandScheduler scheduler;
    private CommandXboxController controller;

    public Drivetrain drivetrain;
    public Vision vision;

    public RobotContainer() {
        scheduler = CommandScheduler.getInstance();
        controller = new CommandXboxController(ControllerConstants.controllerID);

        drivetrain = new Drivetrain(
            DrivetrainConstants.drivetrainConfig,
            DrivetrainConstants.frontLeftConfig, DrivetrainConstants.frontRightConfig,
            DrivetrainConstants.backLeftConfig, DrivetrainConstants.backRightConfig
        );

        vision = new Vision();

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.setFOCSpeedsCmd(
                () -> -controller.getLeftY() * DrivetrainConstants.maxSpeed,
                () -> -controller.getLeftX() * DrivetrainConstants.maxSpeed,
                () -> -controller.getRightX() * DrivetrainConstants.maxAngularSpeed
            )
        );
        
        controller.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        controller.rightBumper()
            .onTrue(drivetrain.setIsAimingCmd(true))
            .onFalse(drivetrain.setIsAimingCmd(false));
    }

    public void periodic() {
        drivetrain.addVisionMeasurements(vision.getPoseEstimates());
        ShotCalculator.updateState(drivetrain.getPose2d(), drivetrain.getSpeeds());
    }

     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(2.189, 4.093, Rotation2d.fromDegrees(0))
    );

    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    
    PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, 
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) 
    );


}
