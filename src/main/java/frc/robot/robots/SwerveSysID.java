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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.FeedforwardConstants;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;

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
                  new FeedbackConfigs().withSensorToMechanismRatio(5.14 * 31.75);
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
                  new FeedbackConfigs().withSensorToMechanismRatio(5.14 * 31.75);
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
                  new FeedbackConfigs().withSensorToMechanismRatio(5.14 * 31.75);
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
                  new FeedbackConfigs().withSensorToMechanismRatio(5.14 * 31.75);
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
            },
            sim -> {},
            new AnalogEncoder(0).get() - 0.638,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.of(new FeedforwardConstants(0, 0.25179, 3.1033, 0.33929)),
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
            },
            sim -> {},
            new AnalogEncoder(1).get() - 0.612,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.of(new FeedforwardConstants(0, 0.36617, 3.2101, 0.26453)),
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
            },
            sim -> {},
            new AnalogEncoder(2).get() - 0.546,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.of(new FeedforwardConstants(0, 0.37473, 3.24, 0.24393)),
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
            },
            sim -> {},
            new AnalogEncoder(3).get() - 0.760,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                  controller.enableContinuousInput(0, 1);
                  controller.setTolerance(0.01);
                }),
            Optional.of(new FeedforwardConstants(0, 0.25159, 3.1999, 0.25259)),
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

  @Override
  public void robotPeriodic() {
    HoundLog.log("Swerve/FLAngle", flAngle);
    HoundLog.log("Swerve/FRAngle", frAngle);
    HoundLog.log("Swerve/BLAngle", blAngle);
    HoundLog.log("Swerve/BRAngle", brAngle);
    HoundLog.log("Swerve/FLDrive", flDrive);
    HoundLog.log("Swerve/FRDrive", frDrive);
    HoundLog.log("Swerve/BLDrive", blDrive);
    HoundLog.log("Swerve/BRDrive", brDrive);
    CommandScheduler.getInstance().run();
  }

  public SysIDCommands getDriveSysIDCommands() {
    return frDrive.getSynchronizedSysIDCommands("Drive SysId", 1, 5, 5, flDrive, blDrive, brDrive);
  }

  public SysIDCommands getAngleSysIDCommands() {
    return frAngle.getSynchronizedSysIDCommands("AngleSysId", 1, 5, 5, flAngle, blAngle, brAngle);
  }
}
