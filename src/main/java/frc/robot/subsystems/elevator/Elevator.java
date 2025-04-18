package frc.robot.subsystems.elevator;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
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

  private final double zeroedPosition = 0;
  private final double stowPosition = 0.02;
  private final double handoffPosition = .79;
  private final double l4Position = 1.125;
  private final double l3Position = 0.526;
  private final double l2Position = 0.106;
  private final double l1Position = 0.02;
  private final double stationPosition = 0.246; // intake from coral station
  private final double groundPosition = 0.091; // ground intake?
  private final double groundAlgaeIntakePosition = 0.326;
  private final double groundAlgaeHoldPosition = 0.376;
  private final double processingPosition = 0.096; // algae processor
  private final double lowAlgaePosition = 0.276; // between l2 and l3
  private final double highAlgaePosition = 0.626; // between l3 and l4
  private Consumer<IdleMode> setIdleMode;

  public Elevator() {
    setIdleMode = (mode -> {});
    upMotor =
        Motor.fromSparkMax(
            ElevatorWiring.ELEVATOR_ID,
            false,
            (SparkMax spark) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.idleMode(IdleMode.kBrake);
              config.encoder.positionConversionFactor(1 / 63.1167979003);
              config.encoder.velocityConversionFactor(1 / 63.1167979003);
              config.smartCurrentLimit(60);
              REVLibError err =
                  spark.configure(
                      config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
              if (!err.equals(REVLibError.kOk)) {
                HoundLog.logFault(
                    "[Elevator] Extension Motor Config Error: " + err.name(), AlertType.kError);
              }
              setIdleMode =
                  mode -> {
                    config.idleMode(mode);
                    REVLibError idlerr =
                        spark.configure(
                            config,
                            ResetMode.kResetSafeParameters,
                            PersistMode.kNoPersistParameters);
                    if (!idlerr.equals(REVLibError.kOk)) {
                      HoundLog.logFault(
                          "[Elevator] Idle Mode Config Error: " + err.name(), AlertType.kError);
                    } else {
                      HoundLog.clearFault("[Elevator] Idle Mode Config Error: " + err.name());
                    }
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
    switchHit = new Trigger(() -> zeroingSwitch.get()).debounce(0.35);
    switchHit.onTrue(
        Commands.runOnce(() -> upMotor.resetPosition(zeroedPosition)).ignoringDisable(true));
    // RobotModeTriggers.teleop()
    //     .onFalse(Commands.runOnce(() -> setIdleMode.accept(IdleMode.kBrake)).ignoringDisable(true));
    RobotModeTriggers.teleop()
        .onTrue(Commands.runOnce(() -> setIdleMode.accept(IdleMode.kCoast)).ignoringDisable(true));
  }

  public Command groundAlgaeIntake() {
    return Commands.runOnce(
            () -> {
              upMotor.setTarget(groundAlgaeIntakePosition);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return upMotor.atTarget();
                }));
  }

  public Command groundAlgaeHold() {
    return Commands.runOnce(() -> upMotor.setTarget(groundAlgaeHoldPosition), this)
        .andThen(Commands.waitUntil(() -> upMotor.atTarget()));
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
                }));
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
    HoundLog.log(path, "Zeroing Switch", zeroingSwitch.get());
  }

  public double getExtension() {
    return upMotor.getPosition();
  }
}
