package frc.robot.subsystems.placer;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.WiringConstants.PlacerWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.ExtendedMath;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Placer extends SubsystemBase implements Loggable {
  private Motor runMotor;

  private Alert configError = new Alert("Placer Config Failed :(", AlertType.kError);
  private final double intakeSpeed = -25;
  public final double coralEjectSpeed = 25;
  public final double algaeEjectSpeed = 38.974;

  public final Trigger hasPieceTrigger =
      new Trigger(
              () -> {
                return runMotor.getTarget() != 0
                    && ExtendedMath.within(runMotor.getVelocity(), 0, 5);
              })
          .debounce(1);

  public Placer() {
    runMotor =
        Motor.fromTalonFX(
            PlacerWiring.PLACER_ID,
            (TalonFX motor) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.Audio.AllowMusicDurDisable = true;
              config.CurrentLimits.StatorCurrentLimitEnable = false;
              config.CurrentLimits.SupplyCurrentLimitEnable = false;
              config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              StatusCode status = StatusCode.StatusCodeNotInitialized;
              for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
                status = motor.getConfigurator().apply(config);
              }
              if (status != StatusCode.OK) configError.set(true);
              else configError.set(false);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.empty(0.5),
            FeedforwardController.forConstantGravity(0, 0.47622, 0.12973, 0.01321),
            TargetType.Velocity);
    runMotor.getSysIDCommands("Placer", 2, 8, 5).putOnDashboard("Placer", this);
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
  public Command eject(double speed) {
    return Commands.runOnce(
            () -> {
              runMotor.setTarget(speed);
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

  //   /**
  //  * @return Ejecting for algae because we need a different speed
  //  */
  // public Command ejectAlgae() {
  //   return Commands.runOnce(
  //           () -> {
  //             runMotor.setTarget(algaeSpeed);
  //           },
  //           this)
  //       .andThen(
  //           Commands.waitUntil(
  //               () -> {
  //                 return runMotor.atTarget();
  //               }));
  // }
}
