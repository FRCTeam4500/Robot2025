package frc.robot.subsystems.swerve;

import static com.revrobotics.spark.SparkBase.PersistMode.*;
import static com.revrobotics.spark.SparkBase.ResetMode.*;
import static frc.robot.WiringConstants.SwerveWiring.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.FeedforwardConstants;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import java.util.Optional;

public class SwerveConstants {
  /** The max speed the robot should travel at */
  public static final ChassisSpeeds MAX_SPEEDS = new ChassisSpeeds(6, 6, 4);

  /** The minimum coefficient for slowmode */
  public static final double MIN_COEFFICIENT = 0.2;

  /** The absolute max acheivable module speed */
  public static final double MAX_MODULE_SPEED = 6;

  /** A coefficient used to correct from translation while rotating */
  public static final double SKEW_COEFFICIENT = -0.129;

  /** The position of the front left module from the robot's center */
  public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(0.2974, 0.2974);

  /** The position of the front right module from the robot's center */
  public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(0.2974, -0.2974);

  /** The position of the back left module from the robot's center */
  public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-0.2974, 0.2974);

  /** The position of the back right module from the robot's center */
  public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-0.2974, -0.2974);

  public static final SwerveModule FRONT_LEFT_MODULE =
      new SwerveModule(
        Motor.fromTalonFX(
          11,
          motor -> {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits =
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true);
            config.MotorOutput =
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive);
            config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(15.5);
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
              status = motor.getConfigurator().apply(config);
            }
          },
          sim -> {},
          0,
          FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
          Optional.of(new FeedforwardConstants(0, 0.24571, 1.8141, 0.16798)),
          Motor.TargetType.Velocity),
          Motor.fromSparkMax(
            9,
            false,
            motor -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
              config
                  .encoder
                  .positionConversionFactor(1.0 / 25)
                  .velocityConversionFactor(1.0 / 25 / 60);
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            },
            sim -> {},
            new AnalogEncoder(0).get() - 0.650,
            FeedbackController.fromPID(
                new PIDController(5, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.of(new FeedforwardConstants(0, 0.25179, 3.1033, 0.33929)),
            Motor.TargetType.Meters));

  public static final SwerveModule FRONT_RIGHT_MODULE =
      new SwerveModule(
        Motor.fromTalonFX(
          10,
          motor -> {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits =
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true);
            config.MotorOutput =
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive);
            config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(15.5);
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
              status = motor.getConfigurator().apply(config);
            }
          },
          sim -> {},
          0,
          FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
          Optional.of(new FeedforwardConstants(0, 0.19746, 1.7841, 0.21748)),
          Motor.TargetType.Velocity),
          Motor.fromSparkMax(
            8,
            false,
            motor -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
              config
                  .encoder
                  .positionConversionFactor(1.0 / 25)
                  .velocityConversionFactor(1.0 / 25 / 60);
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            },
            sim -> {},
            new AnalogEncoder(1).get() - 0.906,
            FeedbackController.fromPID(
                new PIDController(5, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.of(new FeedforwardConstants(0, 0.36617, 3.2101, 0.26453)),
            Motor.TargetType.Meters));

  public static final SwerveModule BACK_LEFT_MODULE =
      new SwerveModule(
        Motor.fromTalonFX(
          19,
          motor -> {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits =
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true);
            config.MotorOutput =
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive);
            config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(15.5);
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
              status = motor.getConfigurator().apply(config);
            }
          },
          sim -> {},
          0,
          FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
          Optional.of(new FeedforwardConstants(0, 0.21224, 1.7981, 0.18871)),
          Motor.TargetType.Velocity),
          Motor.fromSparkMax(
            18,
            false,
            motor -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
              config
                  .encoder
                  .positionConversionFactor(1.0 / 25)
                  .velocityConversionFactor(1.0 / 25 / 60);
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            },
            sim -> {},
            new AnalogEncoder(2).get() - 0.001,
            FeedbackController.fromPID(
                new PIDController(5, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.of(new FeedforwardConstants(0, 0.37473, 3.24, 0.24393)),
            Motor.TargetType.Meters));

  public static final SwerveModule BACK_RIGHT_MODULE =
      new SwerveModule(
        Motor.fromTalonFX(
          7,
          motor -> {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits =
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true);
            config.MotorOutput =
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive);
            config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(15.5);
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
              status = motor.getConfigurator().apply(config);
            }
          },
          sim -> {},
          0,
          FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
          Optional.of(new FeedforwardConstants(0, 0.2177, 1.8156, 0.12033)),
          Motor.TargetType.Velocity),
          Motor.fromSparkMax(
            6,
            false,
            motor -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
              config
                  .encoder
                  .positionConversionFactor(1.0 / 25)
                  .velocityConversionFactor(1.0 / 25 / 60);
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            },
            sim -> {},
            new AnalogEncoder(3).get() - 0.854,
            FeedbackController.fromPID(
                new PIDController(5, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.of(new FeedforwardConstants(0, 0.25159, 3.1999, 0.25259)),
            Motor.TargetType.Meters));
}
