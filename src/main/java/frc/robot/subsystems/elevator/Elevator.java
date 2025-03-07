package frc.robot.subsystems.elevator;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.WiringConstants.ElevatorWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.function.Consumer;

public class Elevator extends SubsystemBase implements Loggable {
  private Motor upMotor;
  private DigitalInput zeroingSwitch;
  private Trigger switchHit;
  public final Trigger armCanIntake =
      new Trigger(
          () -> {
            return upMotor.getPosition() > 0.6;
          });

  private Alert configError = new Alert("Elevator Config Failed :(", AlertType.kError);
  private Alert idleError = new Alert("Elevator Idle Config Failed :(", AlertType.kError);

  private final double zeroedPosition = 0;
  private final double stowPosition = 0.02;
  private final double handoffPosition = .735;
  private final double l4Position = 1.0;
  private final double l3Position = 0.45;
  private final double l2Position = 0.02;
  private final double l1Position = 0.02;
  private final double stationPosition = 0.15; // intake from coral station
  private final double groundPosition = 0.05; // ground intake?
  private final double groundAlgaePosition = 0.25;
  private final double processingPosition = 0.02; // algae processor
  private final double lowAlgaePosition = 0.2; // between l2 and l3
  private final double highAlgaePosition = 0.55; // between l3 and l4
  private Consumer<IdleMode> setIdleMode;

  public Elevator() {
    setIdleMode = (mode -> {});
    upMotor =
        Motor.fromSparkMax(
            ElevatorWiring.ELEVATOR_ID,
            false,
            (SparkMax spark) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.idleMode(IdleMode.kCoast);
              config.encoder.positionConversionFactor(1 / 63.1167979003);
              config.encoder.velocityConversionFactor(1 / 63.1167979003);
              config.smartCurrentLimit(60);
              REVLibError err =
                  spark.configure(
                      config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
              if (!err.equals(REVLibError.kOk)) {
                configError.setText("Elevator Config Error: " + err.name());
                configError.set(true);
              } else configError.set(false);
              setIdleMode =
                  mode -> {
                    config.idleMode(mode);
                    REVLibError idlerr =
                        spark.configure(
                            config,
                            ResetMode.kResetSafeParameters,
                            PersistMode.kNoPersistParameters);
                    if (!idlerr.equals(REVLibError.kOk)) {
                      idleError.setText("Elevator Idle Config Error: " + idlerr.name());
                      idleError.set(true);
                    } else idleError.set(false);
                  };
            },
            (FeedforwardSim sim) -> {
              sim.withHardstops(0, 1);
            },
            0,
            FeedbackController.fromPID(
                new PIDController(75, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(0.05);
                }),
            FeedforwardController.forConstantGravity(0.775, 0.21877, 8.0517, 2.143),
            TargetType.Position);
    zeroingSwitch = new DigitalInput(ElevatorWiring.ZEROING_CHANNEL);
    upMotor.getSysIDCommands("Elevator", 0.5, 2, 5).putOnDashboard("Elevator", this);
    upMotor.setMaxNegativeVoltage(-10);
    switchHit = new Trigger(() -> !zeroingSwitch.get());
    switchHit.onTrue(
        Commands.runOnce(() -> upMotor.resetPosition(zeroedPosition)).ignoringDisable(true));
    RobotModeTriggers.autonomous()
        .onFalse(Commands.runOnce(() -> setIdleMode.accept(IdleMode.kBrake)).ignoringDisable(true));
    RobotModeTriggers.disabled()
        .onFalse(Commands.runOnce(() -> setIdleMode.accept(IdleMode.kCoast)).ignoringDisable(true));
  }

  public Command groundAlgae() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(groundAlgaePosition);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that stows the elevator
   */
  public Command stow() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(stowPosition);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }))
        .andThen(Commands.runOnce(() -> upMotor.setVoltage(0)));
  }

  /**
   * @return A command that makes the elevator go up to the fourth level
   */
  public Command level4() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(l4Position);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that makes the elevator go to the third level
   */
  public Command level3() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(l3Position);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that moves the elevator up to the second level
   */
  public Command level2() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(l2Position);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }))
        .andThen(Commands.runOnce(() -> upMotor.setVoltage(0)));
  }

  /**
   * @return A command that moves the elevator up to the first level
   */
  public Command level1() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(l1Position);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }))
        .andThen(Commands.runOnce(() -> upMotor.setVoltage(0)));
  }

  /**
   * @return A command that moves the elevator down to handoff
   */
  public Command handoff() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(handoffPosition);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that moves the elevator to the level of the lower algae
   */
  public Command lowAlgae() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(lowAlgaePosition);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that moves the elevator to the level of the higher algae
   */
  public Command highAlgae() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(highAlgaePosition);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that moves the elevator to the level of the higher algae
   */
  public Command stationPickup() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(stationPosition);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that moves the elevator to the level of the higher algae
   */
  public Command groundPickup() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(groundPosition);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  /**
   * @return A command that moves the elevator to the level of the higher algae
   */
  public Command processing() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(processingPosition);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }))
        .andThen(Commands.runOnce(() -> upMotor.setVoltage(0)));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Up Motor", upMotor);
    HoundLog.log(path, "Zeroing Switch", !zeroingSwitch.get());
  }

  public double getExtension() {
    return upMotor.getPosition();
  }
}
