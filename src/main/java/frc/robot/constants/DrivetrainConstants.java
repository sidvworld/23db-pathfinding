// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.units.measure.*;

public class DrivetrainConstants {
    public static final double translationP = 0, translationI = 0, translationD = 0;
    public static final double headingP = 8, headingI = 0, headingD = 0;

    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(2.66).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement driveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement steerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

    private static final Current slipCurrent = Amps.of(120);

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    private static final Pigeon2Configuration pigeonConfigs = null;

    public static final CANBus canBus = new CANBus("CANivore", "./logs/example.hoot");

    public static final LinearVelocity speedAt12Volts = MetersPerSecond.of(4.58);

    public static final double maxSpeed = speedAt12Volts.in(MetersPerSecond);
    public static final double maxAngularSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private static final double coupleRatio = 3.5714285714285716;
    private static final double driveGearRatio = 6.746031746031747;
    private static final double steerGearRatio = 21.428571428571427;
    private static final Distance wheelRadius = Inches.of(2);

    private static final boolean invertLeftSide = false;
    private static final boolean invertRightSide = true;

    private static final int pigeonId = 0;

    private static final MomentOfInertia steerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia driveInertia = KilogramSquareMeters.of(0.01);

    private static final Voltage steerFrictionVoltage = Volts.of(0.2);
    private static final Voltage driveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants drivetrainConfig =
        new SwerveDrivetrainConstants()
            .withCANBusName(canBus.getName())
            .withPigeon2Id(pigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(driveGearRatio)
            .withSteerMotorGearRatio(steerGearRatio)
            .withCouplingGearRatio(coupleRatio)
            .withWheelRadius(wheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSlipCurrent(slipCurrent)
            .withSpeedAt12Volts(speedAt12Volts)
            .withDriveMotorType(driveMotorType)
            .withSteerMotorType(steerMotorType)
            .withFeedbackSource(steerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(steerInertia)
            .withDriveInertia(driveInertia)
            .withSteerFrictionVoltage(steerFrictionVoltage)
            .withDriveFrictionVoltage(driveFrictionVoltage);

    private static final int frontLeftDriveMotorId = 7;
    private static final int frontLeftSteerMotorId = 8;
    private static final int frontLeftEncoderId = 54;
    private static final Angle frontLeftEncoderOffset = Rotations.of(0.402587890625);
    private static final boolean frontLeftSteerMotorInverted = true;
    private static final boolean frontLeftEncoderInverted = false;

    private static final Distance frontLeftXPos = Inches.of(11.25);
    private static final Distance frontLeftYPos = Inches.of(10.25);

    private static final int frontRightDriveMotorId = 3;
    private static final int frontRightSteerMotorId = 4;
    private static final int frontRightEncoderId = 51;
    private static final Angle frontRightEncoderOffset = Rotations.of(-0.437744140625);
    private static final boolean frontRightSteerMotorInverted = true;
    private static final boolean frontRightEncoderInverted = false;

    private static final Distance frontRightXPos = Inches.of(11.25);
    private static final Distance frontRightYPos = Inches.of(-10.25);

    private static final int backLeftDriveMotorId = 1;
    private static final int backLeftSteerMotorId = 2;
    private static final int backLeftEncoderId = 53;
    private static final Angle backLeftEncoderOffset = Rotations.of(-0.39306640625);
    private static final boolean backLeftSteerMotorInverted = true;
    private static final boolean backLeftEncoderInverted = false;

    private static final Distance backLeftXPos = Inches.of(-11.25);
    private static final Distance backLeftYPos = Inches.of(10.25);

    private static final int backRightDriveMotorId = 5;
    private static final int backRightSteerMotorId = 6;
    private static final int backRightEncoderId = 52;
    private static final Angle backRightEncoderOffset = Rotations.of(0.044921875);
    private static final boolean backRightSteerMotorInverted = true;
    private static final boolean backRightEncoderInverted = false;

    private static final Distance backRightXPos = Inches.of(-11.25);
    private static final Distance backRightYPos = Inches.of(-10.25);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftConfig =
        constantCreator.createModuleConstants(
            frontLeftSteerMotorId, frontLeftDriveMotorId, frontLeftEncoderId, frontLeftEncoderOffset,
            frontLeftXPos, frontLeftYPos, invertLeftSide, frontLeftSteerMotorInverted, frontLeftEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightConfig =
        constantCreator.createModuleConstants(
            frontRightSteerMotorId, frontRightDriveMotorId, frontRightEncoderId, frontRightEncoderOffset,
            frontRightXPos, frontRightYPos, invertRightSide, frontRightSteerMotorInverted, frontRightEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftConfig =
        constantCreator.createModuleConstants(
            backLeftSteerMotorId, backLeftDriveMotorId, backLeftEncoderId, backLeftEncoderOffset,
            backLeftXPos, backLeftYPos, invertLeftSide, backLeftSteerMotorInverted, backLeftEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightConfig =
        constantCreator.createModuleConstants(
            backRightSteerMotorId, backRightDriveMotorId, backRightEncoderId, backRightEncoderOffset,
            backRightXPos, backRightYPos, invertRightSide, backRightSteerMotorInverted, backRightEncoderInverted
        );
}
