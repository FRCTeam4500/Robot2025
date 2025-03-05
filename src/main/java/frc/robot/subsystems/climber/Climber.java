package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
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

public class Climber extends SubsystemBase implements Loggable {

  private Motor winchMotor;

  private final double latchPosition = 40;
  private final double readyPosition = 140;

  private Alert configError = new Alert("Climber Config Failed :(", AlertType.kError);

  public Climber() {
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
                  config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                  StatusCode status = StatusCode.StatusCodeNotInitialized;
                  for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                    status = motor.getConfigurator().apply(config);
                  }
                  if (status != StatusCode.OK) {
                    configError.setText("Climber Config Error: " + status.name());
                    configError.set(true);
                  }
                  else configError.set(false);
                },
                (FeedforwardSim sim) -> {},
                0.0,
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
    winchMotor.useThroughBoreEncoder(
        ClimberWiring.ENCODER_CHANNEL, false, (21.772 / 360.) - (47. / 360.));
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
    return Commands.runOnce(() -> winchMotor.setTarget(readyPosition))
        .andThen(
            Commands.runOnce(
                () -> {
                  winchMotor.setVoltage(8.);
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
              winchMotor.setTarget(latchPosition);
            },
            this)
        .andThen(
            Commands.runOnce(
                () -> {
                  winchMotor.setVoltage(-3);
                }))
        .andThen(
            Commands.waitUntil(
                () -> {
                  return winchMotor.getPosition() <= latchPosition && winchMotor.getPosition() > 0;
                }))
        .andThen(Commands.runOnce(() -> winchMotor.setVoltage(0)));
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
