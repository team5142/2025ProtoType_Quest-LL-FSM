// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class EnabledSubsystems {

        public static final boolean chasis = true;
        public static final boolean ll = true;
        public static final boolean questnav = true;
    }

    public static final class DebugTelemetrySubsystems {
        public static final boolean odometry = true;
        public static final boolean imu = true;
        public static final boolean chassis = true;
        public static final boolean ll = true;
        public static final boolean questnav = true;
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;

        public static class OIContants {

            public static enum ControllerDeviceType {
                LOGITECH,
                PS5,
                XBOX, // RightJ F/B, LeftJ L/R, L2/R2 - rotation
                XBOX_ONEDRIVE // RIghtJ F/B/L/R, LeftJ - rotation
            }

            public static record ControllerDevice(int portNumber, ControllerDeviceType controllerDeviceType,
                    double deadbandX, double deadbandY, double deadbandOmega,
                    boolean cubeControllerLeftStick, boolean cubeControllerRightStick) {
            }

            public static ControllerDevice XBOX_CONTROLLER = new ControllerDevice(
                    0,
                    ControllerDeviceType.XBOX,
                    0.1,
                    0.1,
                    0.1,
                    false,
                    false);

        }

        /** Swerve-wide constants and module mappings */
        public static final class SwerveConstants {

            public static final double CHASSIS_POSE_HISTORY_TIME = 0.6; // seconds

            public static final boolean CTR_ODOMETRY_UPDATE_FROM_QUEST = true;

            public static final double MaxSpeed = 5.21; // m/s
            public static final double MaxAngularRate = 4.71238898038469; // rad/s
            public static final double DeadbandRatioLinear = 0.15; // determined by calibration method
            public static final double DeadbandRatioAngular = 0.15; // determined by calibration method

            public static final CANBus kCANBus = new CANBus("Canivore", "./logs/example.hoot"); // YOUR CANIVORE NAME

            public static final Pigeon2Configuration pigeonConfigs = null;
            // Steer motor PID gains - TUNED from TunerX project
            private static final Slot0Configs steerGains = new Slot0Configs()
                    .withKP(50).withKI(0).withKD(0.2)
                    .withKS(0).withKV(0).withKA(0);
            // Drive motor PID gains - TUNED from TunerX project
            private static final Slot0Configs driveGains = new Slot0Configs()
                    .withKP(3).withKI(0).withKD(0)
                    .withKS(0).withKV(0).withKA(0);
            // Rotation controller PID gains - TUNED from TunerX project
            public static final double kRot_kP = 13.616;
            public static final double kRot_kI = 0.0;
            public static final double kRot_kD = 0.1;

            // Translation controller PID gains - PathPlanner auto following
            public static final double kTrans_kP = 10.0;
            public static final double kTrans_kI = 0.0;
            public static final double kTrans_kD = 0.0;
            public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    // Swerve azimuth does not require much torque output, so we can set a
                                    // relatively low
                                    // stator current limit to help avoid brownouts without impacting performance.
                                    .withStatorCurrentLimit(Amps.of(60))
                                    .withStatorCurrentLimitEnable(true))
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withNeutralMode(NeutralModeValue.Brake));
            public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    .withSupplyCurrentLimit(Amps.of(60))
                                    .withSupplyCurrentLimitEnable(true)
                                    .withStatorCurrentLimit(Amps.of(80))
                                    .withStatorCurrentLimitEnable(true))
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withNeutralMode(NeutralModeValue.Brake));
            public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

            // Added from original TunerConstants (auto-merged):

            // Auto-merged constant declarations from original TunerConstants:
            public static final double kCoupleRatio = 3.5714285714285716;
            public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
            public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);
            public static final double kDriveGearRatio = 6.122448979591837;
            public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
            public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;

            public static final int kPigeonId = 14; // YOUR PIGEON ID

            public static final Current kSlipCurrent = Amps.of(120.0);
            public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.21);
            public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
            public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;
            public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
            public static final double kSteerGearRatio = 21.428571428571427;
            public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
            public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
            public static final Distance kWheelRadius = Inches.of(1.959);

            public static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withCouplingGearRatio(kCoupleRatio)
                    .withWheelRadius(kWheelRadius)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                    .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                    .withSlipCurrent(kSlipCurrent)
                    .withSpeedAt12Volts(kSpeedAt12Volts)
                    .withDriveMotorType(kDriveMotorType)
                    .withSteerMotorType(kSteerMotorType)
                    .withFeedbackSource(kSteerFeedbackType)
                    .withDriveMotorInitialConfigs(driveInitialConfigs)
                    .withSteerMotorInitialConfigs(steerInitialConfigs)
                    .withEncoderInitialConfigs(encoderInitialConfigs)
                    .withSteerInertia(kSteerInertia)
                    .withDriveInertia(kDriveInertia)
                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
                    .withDriveFrictionVoltage(kDriveFrictionVoltage);
            public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                    .withCANBusName(kCANBus.getName())
                    .withPigeon2Id(kPigeonId)
                    .withPigeon2Configs(pigeonConfigs);

            public static record SwerveModuleConstantsRecord(int driveMotorID, int angleMotorID, int cancoderID,
                    double angleOffset,
                    boolean driveMotorInverted, boolean angleMotorInverted, boolean cancoderInverted) {
            }

            // 5142 SWERVE CONSTANTS

            // Front Left Module - UPDATE OFFSET FROM NEW TUNERX
            public static final SwerveModuleConstantsRecord MOD0 = new SwerveModuleConstantsRecord(
                    2,
                    1,
                    9,
                    0.444580078125, // ← CHANGE from 0.121094 to NEW TunerX value
                    false, // driveMotorInverted (LEFT SIDE)
                    true, // angleMotorInverted
                    false // canCoderInverted
            );

            // Front Right Module - Minor adjustment
            public static final SwerveModuleConstantsRecord MOD1 = new SwerveModuleConstantsRecord(
                    4,
                    3,
                    10,
                    0.210205078125, // ← CHANGE from 0.235107 (9° difference)
                    true, // driveMotorInverted (RIGHT SIDE)
                    true, // angleMotorInverted
                    false // canCoderInverted
            );

            // Back Left Module - Minor adjustment
            public static final SwerveModuleConstantsRecord MOD2 = new SwerveModuleConstantsRecord(
                    8,
                    7,
                    12,
                    -0.29736328125, // ← CHANGE from -0.287354 (3.6° difference)
                    false, // driveMotorInverted (LEFT SIDE)
                    true, // angleMotorInverted
                    false // canCoderInverted
            );

            // Back Right Module - Basically unchanged
            public static final SwerveModuleConstantsRecord MOD3 = new SwerveModuleConstantsRecord(
                    6,
                    5,
                    11,
                    0.3837890625, // ← CHANGE from 0.381104 (1° difference)
                    true, // driveMotorInverted (RIGHT SIDE)
                    true, // angleMotorInverted
                    false // canCoderInverted
            );
        }
    }

    public static final class PathPlannerConstants {
        public static final boolean shouldFlipTrajectoryOnRed = true;
    }

}