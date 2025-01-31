package frc.robot.subsystems.placer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.WiringConstants.PlacerWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.FeedforwardConstants;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.ExtendedMath;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.Optional;

public class Placer extends SubsystemBase implements Loggable {
  private Motor runMotor;

  private final double intakeSpeed = -25;
  private final double ejectSpeed = 25;

  public final Trigger hasPieceTrigger =
      new Trigger(
              () -> {
                return runMotor.getTarget() != 0
                    && ExtendedMath.within(runMotor.getVelocity(), 0, 5);
              })
          .debounce(2);

  public Placer() {
    runMotor =
        Motor.fromTalonFX(
            PlacerWiring.PLACER_ID,
            (TalonFX fx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.Audio.AllowMusicDurDisable = true;
              config.CurrentLimits.StatorCurrentLimit = 40;
              config.CurrentLimits.StatorCurrentLimitEnable = false;
              config.CurrentLimits.SupplyCurrentLimitEnable = false;
              config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              fx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(0.5);
                }),
            Optional.of(new FeedforwardConstants(0, 0.47622, 0.12973, 0.01321)),
            TargetType.Velocity);
  }

  /**
   * @return A command that stowps the grabber
   */
  public Command stop() {
    return Commands.runOnce(
            () -> {
              runMotor.setTarget(0);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return runMotor.atTarget();
                }));
  }

  /**
   * @return A command that makes the intaker suck
   */
  public Command intake() {
    return Commands.runOnce(
            () -> {
              runMotor.setTarget(intakeSpeed);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return runMotor.atTarget();
                }));
  }

  /**
   * @return A command that makes the shooter?? intaker?? spit
   */
  public Command eject() {
    return Commands.runOnce(
            () -> {
              runMotor.setTarget(ejectSpeed);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return runMotor.atTarget();
                }));
  }

  public void log(String path) {
    HoundLog.log(path, "Speed Motor", runMotor);
    HoundLog.log(path, "Intaked", hasPieceTrigger.getAsBoolean());
  }
}
