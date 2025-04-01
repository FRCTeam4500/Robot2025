package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.WiringConstants.ClimberWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.function.Consumer;

public class Climber extends SubsystemBase implements Loggable {

  private Motor winchMotor;
  private Consumer<NeutralModeValue> setIdleMode;

  private final double latchPosition = 33;
  private final double readyPosition = 150;

  public Climber() {
    setIdleMode = (mode) -> {};
    winchMotor =
        Motor.fromTalonFX(
                ClimberWiring.CLIMBER_ID,
                (TalonFX motor) -> {
                  TalonFXConfiguration config = new TalonFXConfiguration();
                  config.Audio.AllowMusicDurDisable = true;
                  config.Feedback.SensorToMechanismRatio = 1;
                  config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                  config.CurrentLimits.StatorCurrentLimit = 60;
                  config.CurrentLimits.StatorCurrentLimitEnable = true;
                  config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                  StatusCode status = StatusCode.StatusCodeNotInitialized;
                  for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                    status = motor.getConfigurator().apply(config);
                  }
                  if (status != StatusCode.OK) {
                    HoundLog.logFault(
                        "[Climber] Winch Motor Config Error: " + status.getName(),
                        AlertType.kError);
                  }
                  setIdleMode =
                      (mode) -> {
                        config.MotorOutput.NeutralMode = mode;
                        StatusCode idleStatus = motor.getConfigurator().apply(config);
                        if(idleStatus != StatusCode.OK) {
                          HoundLog.logFault(
                            "[Climber] Winch Motor Config Error: " + idleStatus.name(),
                            AlertType.kError
                          );
                        };
                      };
                },
                (FeedforwardSim sim) -> {},
                40,
                FeedbackController.fromPID(
                    0.1,
                    0,
                    0,
                    pid -> {
                      pid.enableContinuousInput(-180, 180);
                    }),
                FeedforwardController.forNone(),
                TargetType.Position)
            .withName("Climber Motor");
    winchMotor.useThroughBoreEncoder(ClimberWiring.ENCODER_CHANNEL, false, 0.177 - 0.25);
    winchMotor.getSysIDCommands("Climber", 1, 1, 5).putOnDashboard("Climber", this);
  }

  public Command pause() {
    return Commands.runOnce(
        () -> {
          winchMotor.setTarget(winchMotor.getPosition());
        },
        this);
  }

  public Command ready() {
    return Commands.runOnce(
            () -> {
              setIdleMode.accept(NeutralModeValue.Coast);
            })
        .andThen(
            Commands.runOnce(
                () -> {
                  winchMotor.setVoltage(10.);
                },
                this))
        .andThen(
            Commands.waitUntil(
                () -> {
                  return winchMotor.getPosition() >= readyPosition;
                }))
        .andThen(Commands.runOnce(() -> winchMotor.setVoltage(0)));
  }

  public Command climb() {
    return Commands.runOnce(
            () -> {
              setIdleMode.accept(NeutralModeValue.Brake);
            },
            this)
        .andThen(
            Commands.runOnce(
                () -> {
                  winchMotor.setVoltage(-10);
                }))
        .andThen(
            Commands.waitUntil(
                () -> {
                  return winchMotor.getPosition() <= latchPosition && winchMotor.getPosition() > 0;
                }))
        .andThen(() -> {winchMotor.setVoltage(0);});
  }

  public Command off() {
    return Commands.runOnce(() -> winchMotor.setVoltage(0), this);
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Tilty Motor", winchMotor);
  }

  public double getAngle() {
    return winchMotor.getPosition();
  }
}
