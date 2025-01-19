package frc.robot.robots;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.HoundLog;

public class SwerveTest extends LoggedRobot {
    private Motor[] driveMotors = new Motor[] {
        Motor.fromTalonFX( // fl
            1, 
            motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(new PIDController(0, 0, 0), pid -> {}), 
            Optional.empty(), 
            TargetType.Velocity
        ),
        Motor.fromTalonFX( // fr
            2, 
            motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(new PIDController(0, 0, 0), pid -> {}), 
            Optional.empty(), 
            TargetType.Velocity
        ),
        Motor.fromTalonFX( // bl
            3, 
            motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(new PIDController(0, 0, 0), pid -> {}), 
            Optional.empty(), 
            TargetType.Velocity
        ),
        Motor.fromTalonFX( // br
            4, 
            motor -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                config.CurrentLimits =
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true);
                config.MotorOutput =
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive);
                config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(12.1908);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(new PIDController(0, 0, 0), pid -> {}), 
            Optional.empty(), 
            TargetType.Velocity
        )
    };
    private Motor[] angleMotors = new Motor[] {
        Motor.fromSparkMax(
            5, 
            true, 
            spark -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25)
                    .velocityConversionFactor(1.0 / 25 / 60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ),
            Optional.empty(),
            TargetType.Position
        ),
        Motor.fromSparkMax(
            6, 
            true, 
            spark -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25)
                    .velocityConversionFactor(1.0 / 25 / 60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ),
            Optional.empty(),
            TargetType.Position
        ),
        Motor.fromSparkMax(
            7, 
            true, 
            spark -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25)
                    .velocityConversionFactor(1.0 / 25 / 60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ),
            Optional.empty(),
            TargetType.Position
        ),
        Motor.fromSparkMax(
            8, 
            true, 
            spark -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.inverted(false).smartCurrentLimit(20).idleMode(IdleMode.kBrake);
                config
                    .encoder
                    .positionConversionFactor(1.0 / 25)
                    .velocityConversionFactor(1.0 / 25 / 60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }, 
            sim -> {}, 
            0, 
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                controller -> {
                    controller.enableContinuousInput(0, 1);
                    controller.setTolerance(0.01);
                }
            ),
            Optional.empty(),
            TargetType.Position
        ),
    };
    private AnalogEncoder[] encoders = new AnalogEncoder[] {
        new AnalogEncoder(0),
        new AnalogEncoder(1),
        new AnalogEncoder(2),
        new AnalogEncoder(3)
    };
    private double[] offsets = new double[] {0, 0, 0, 0};    
    public SwerveTest() {
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        HoundLog.log("Drive Motors", driveMotors);
        HoundLog.log("Angle Motors", angleMotors);
        HoundLog.log("Encoder Readings", new double[] {
            encoders[0].get() - offsets[0],
            encoders[1].get() - offsets[1],
            encoders[2].get() - offsets[2],
            encoders[3].get() - offsets[3],
        });
    }
}
