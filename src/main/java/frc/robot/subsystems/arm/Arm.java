package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.WiringConstants.ArmWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.subsystems.orchestra.Orc;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Arm extends SubsystemBase implements Loggable {
  private Motor tiltMotor;

  private final double startAngle = 90.851;
  private final double stowAngle = 75;
  private double placeL4Angle = 55;
  private DoubleSupplier l4AngleGetter = () -> placeL4Angle;
  private final double placeL3Angle = 60;
  private final double placeL2Angle = 73.15;
  private final double placeL1Angle = 40;
  private final double handoffAngle = -85;
  private final double stationAngle = 70;
  private final double groundAngle = -21.415;
  private final double climbAngle = -20;
  private final double dislodgeAngle = 40;
  private final double algaeGroundIntakeAngle = -20;
  private final double algaeGroundHoldAngle = -24.53;

  public final Trigger canMoveElevator =
      new Trigger(() -> tiltMotor.getPosition() > -25 && tiltMotor.getPosition() < 77);

  public Arm() {
    tiltMotor =
        Motor.fromTalonFX(
                ArmWiring.ARM_ID,
                motor -> {
                  TalonFXConfiguration config = new TalonFXConfiguration();
                  config.Audio.AllowMusicDurDisable = true;
                  config.Feedback.SensorToMechanismRatio = 55.0 / 360;
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
                        "[Arm] Tilt Motor Config Error: " + status.getName(), AlertType.kError);
                  } else {
                    Orc.addMotor(motor);
                  }
                },
                sim -> {
                  sim.withHardstops(handoffAngle, startAngle);
                },
                90,
                FeedbackController.fromPID(
                    new PIDController(0.06, 0, 0),
                    (PIDController pid) -> {
                      pid.setTolerance(2);
                    }),
                FeedforwardController.forArmGravity(0.35, 0.034937, 0.015511, 0.0042897),
                TargetType.Position)
            .withName("Arm Motor");

    tiltMotor.useThroughBoreEncoder(ArmWiring.ENCODER_CHANNEL, true, .218);
    tiltMotor.getSysIDCommands("Arm", 0.25, 0.5, 4).putOnDashboard("Arm", this);
    SmartDashboard.putData(
      new Sendable() {

        @Override
        public void initSendable(SendableBuilder builder) {
          builder.addDoubleProperty("Editable Arm L4 Position", () -> placeL4Angle, (d) -> placeL4Angle = d);
        }
        
      }
    );
  }

  public Command stow() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(stowAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command ground() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(groundAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command climb() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(climbAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command algaeGroundIntake() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(algaeGroundIntakeAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command algaeGroundHold() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(algaeGroundHoldAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command dislodge() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(dislodgeAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command handoff() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(handoffAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command placeL4() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(l4AngleGetter.getAsDouble());
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command placeL3() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(placeL3Angle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command placeL2() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(placeL2Angle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command placeL1() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(placeL1Angle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  public Command stationPickup() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(stationAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltMotor.atTarget();
                }));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Tilt Motor", tiltMotor);
    HoundLog.log(path, "Can Move Elevator", canMoveElevator.getAsBoolean());
  }

  public double getAngle() {
    return tiltMotor.getPosition();
  }
}
