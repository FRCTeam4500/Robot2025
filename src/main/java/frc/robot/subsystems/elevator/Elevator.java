package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.WiringConstants.ElevatorWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.FeedforwardConstants;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.Optional;

public class Elevator extends SubsystemBase implements Loggable {
  private Motor upMotor;
  public final MechanismLigament2d mech;
  public final MechanismLigament2d armHolder;

  private final double stowPosition = 0;
  private final double handoffPosition = .5;
  private final double l4Position = 1;
  private final double l3Position = 0.75;
  private final double l2Position = 0.5;
  private final double l1Position = 0;
  private final double stationPosition = 0; // intake from coral station
  private final double groundPosition = 0; // ground intake?
  private final double processingPosition = 0; // algae processor
  private final double lowAlgaePosition = 0; // between l2 and l3
  private final double highAlgaePosition = 0; // between l3 and l4

  public Elevator() {
    upMotor =
        Motor.fromSparkMax(
            ElevatorWiring.ELEVATOR_ID,
            false,
            (SparkMax spark) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.idleMode(IdleMode.kCoast);
                config.encoder.positionConversionFactor(1 / 216.5);
                config.encoder.velocityConversionFactor(1 / 216.5 / 60);
                config.smartCurrentLimit(60);
                spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim sim) -> {
              sim.withHardstops(0, 1);
            },
            0,
            FeedbackController.fromPID(
                new PIDController(15, 0, 0), 
                (PIDController pid) -> {
                    pid.setTolerance(0.05);
                }
            ),
            Optional.of(new FeedforwardConstants(0.15472, 0.098108, 27.673, 3.4548)), 
            TargetType.Meters);
    mech = new MechanismLigament2d("Elevator", 0, 90);
    armHolder = new MechanismLigament2d("Arm Holder", 0.1, -90);
    mech.append(armHolder);
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
                }));
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
                }));
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
                }));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Up Motor", upMotor);
    mech.setLength(upMotor.getPosition() + 0.1);
  }
}
