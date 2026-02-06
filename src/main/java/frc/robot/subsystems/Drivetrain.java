// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShotCalculator;
import frc.robot.Utilities;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.FieldConstants;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private SwerveRequest.FieldCentric focRequest;
    private SwerveRequest.RobotCentric rocRequest;

    private PIDController translationPID, headingPID;

    private boolean isAiming;
    RobotConfig config;

    public Drivetrain(SwerveDrivetrainConstants drivetrainConfig, SwerveModuleConstants<?, ?, ?>... moduleConfigs) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfig, moduleConfigs);

        focRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DrivetrainConstants.maxSpeed * ControllerConstants.deadband)
            .withRotationalDeadband(DrivetrainConstants.maxAngularSpeed * ControllerConstants.deadband);

        rocRequest = new SwerveRequest.RobotCentric();

        translationPID = new PIDController(DrivetrainConstants.translationP, DrivetrainConstants.translationI, DrivetrainConstants.translationD);
        headingPID = new PIDController(DrivetrainConstants.headingP, DrivetrainConstants.headingI, DrivetrainConstants.headingD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        

    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose2d, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setROCSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(DrivetrainConstants.translationP, DrivetrainConstants.translationI, DrivetrainConstants.translationD), // Translation PID constants
                    new PIDConstants(DrivetrainConstants.headingP, DrivetrainConstants.headingI, DrivetrainConstants.headingD) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

    
    
    public double calcAimingPID(double targetHeading) {
        double power = headingPID.calculate(getRotation2d().getRadians(), targetHeading);
        return MathUtil.clamp(power, -DrivetrainConstants.maxAngularSpeed, DrivetrainConstants.maxAngularSpeed);
    }

    public Rotation2d getRotation2d() {
        return getState().RawHeading;
    }

    public Pose2d getPose2d() {
        return getState().Pose;
    }

    public ChassisSpeeds getSpeeds() {
        return getState().Speeds;
    }

    public void addVisionMeasurement(Pose2d rawEstimate) {
        if (Utilities.isValidPose(rawEstimate)) {
            Pose2d estimate = new Pose2d(rawEstimate.getTranslation(), getRotation2d());
            addVisionMeasurement(estimate, Utils.getCurrentTimeSeconds());
        }
    }

    public void addVisionMeasurements(Pose2d[] estimates) {
        for (Pose2d estimate : estimates) {
            addVisionMeasurement(estimate);
        }
    }

    public void setROCSpeeds(ChassisSpeeds speeds) {
        setControl(rocRequest
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vxMetersPerSecond)
            .withRotationalRate(isAiming ? calcAimingPID(ShotCalculator.targetHeading) : speeds.vxMetersPerSecond)
        );
    }

    public Command setFOCSpeedsCmd(DoubleSupplier speedX, DoubleSupplier speedY, DoubleSupplier angularSpeed) {
        return run(() -> setControl(focRequest
                .withVelocityX(speedX.getAsDouble())
                .withVelocityY(speedY.getAsDouble())
                .withRotationalRate(isAiming ? calcAimingPID(ShotCalculator.targetHeading) : angularSpeed.getAsDouble())
            )
        );
    }

    public Command setIsAimingCmd(boolean value) {
        return runOnce(() -> isAiming = value);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                Utilities.getAlliance() == Alliance.Red ? FieldConstants.redPerspective : FieldConstants.bluePerspective
            );
        }
    }

    public Command watchyojetbruh() {
        Pose2d targetPose = new Pose2d(2.189, 4.093, Rotation2d.fromDegrees(180));

    // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands

        
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );
 
    }
    

    


}

   

