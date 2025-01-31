package frc.robot.robots;

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
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.Motor;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.SysIDCommands;
import java.util.Optional;

public class SwerveSysID extends LoggedRobot {
  Motor flDrive;
  Motor flAngle;
  Motor frDrive;
  Motor frAngle;
  Motor blDrive;
  Motor blAngle;
  Motor brDrive;
  Motor brAngle;

  public SwerveSysID() {
    flDrive =
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
              config.Feedback =
                  new FeedbackConfigs().withSensorToMechanismRatio(5.14 * 0.1016 * Math.PI);
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
            },
            sim -> {},
            0,
            FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
            Optional.empty(),
            Motor.TargetType.Velocity);

    frDrive =
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
                      .withInverted(InvertedValue.Clockwise_Positive);
              config.Feedback =
                  new FeedbackConfigs().withSensorToMechanismRatio(5.14 * 0.1016 * Math.PI);
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
            },
            sim -> {},
            0,
            FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
            Optional.empty(),
            Motor.TargetType.Velocity);

    flDrive =
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
              config.Feedback =
                  new FeedbackConfigs().withSensorToMechanismRatio(5.14 * 0.1016 * Math.PI);
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
            },
            sim -> {},
            0,
            FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
            Optional.empty(),
            Motor.TargetType.Velocity);

    blDrive =
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
              config.Feedback =
                  new FeedbackConfigs().withSensorToMechanismRatio(5.14 * 0.1016 * Math.PI);
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
            },
            sim -> {},
            0,
            FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
            Optional.empty(),
            Motor.TargetType.Velocity);

    brDrive =
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
                      .withInverted(InvertedValue.Clockwise_Positive);
              config.Feedback =
                  new FeedbackConfigs().withSensorToMechanismRatio(5.14 * 0.1016 * Math.PI);
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
            },
            sim -> {},
            0,
            FeedbackController.fromPID(new PIDController(0, 0, 0), controller -> {}),
            Optional.empty(),
            Motor.TargetType.Velocity);

    flAngle =
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
              AnalogEncoder absoluteEncoder = new AnalogEncoder(0);
              motor.getEncoder().setPosition(absoluteEncoder.get() - 0.805);
              absoluteEncoder.close();
            },
            sim -> {},
            0,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.empty(),
            Motor.TargetType.Meters);

    frAngle =
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
              AnalogEncoder absoluteEncoder = new AnalogEncoder(1);
              motor.getEncoder().setPosition(absoluteEncoder.get() - 0.805);
              absoluteEncoder.close();
            },
            sim -> {},
            0,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.empty(),
            Motor.TargetType.Meters);

    blAngle =
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
              AnalogEncoder absoluteEncoder = new AnalogEncoder(2);
              motor.getEncoder().setPosition(absoluteEncoder.get() - 0.805);
              absoluteEncoder.close();
            },
            sim -> {},
            0,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.empty(),
            Motor.TargetType.Meters);

    brAngle =
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
              AnalogEncoder absoluteEncoder = new AnalogEncoder(3);
              motor.getEncoder().setPosition(absoluteEncoder.get() - 0.805);
              absoluteEncoder.close();
            },
            sim -> {},
            0,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.empty(),
            Motor.TargetType.Meters);

    SysIDCommands driveSysId = getDriveSysIDCommands();
    SysIDCommands angleSysId = getAngleSysIDCommands();
    SmartDashboard.putData("Drive Dynamic Forward", driveSysId.dynamicForward());
    SmartDashboard.putData("Drive Dynamic Reverse", driveSysId.dynamicReverse());
    SmartDashboard.putData("Drive Quasistatic Forward", driveSysId.quasistaticForward());
    SmartDashboard.putData("Drive Quasistatic Reverse", driveSysId.quasistaticReverse());
    SmartDashboard.putData("Angle Dynamic Forward", angleSysId.dynamicForward());
    SmartDashboard.putData("Angle Dynamic Reverse", angleSysId.dynamicReverse());
    SmartDashboard.putData("Angle Quasistatic Forward", angleSysId.quasistaticForward());
    SmartDashboard.putData("Angle Quasistatic Reverse", angleSysId.quasistaticReverse());
  }

  public SysIDCommands getDriveSysIDCommands() {
    return frDrive.getSynchronizedSysIDCommands("Drive SysId", 1, 5, 5, flDrive, blDrive, brDrive);
  }

  public SysIDCommands getAngleSysIDCommands() {
    return frAngle.getSynchronizedSysIDCommands("AngleSysId", 1, 5, 5, flAngle, blAngle, brAngle);
  }
}
