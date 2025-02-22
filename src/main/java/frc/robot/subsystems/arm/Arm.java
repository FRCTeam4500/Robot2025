package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.WiringConstants.ArmWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Arm extends SubsystemBase implements Loggable {
  private Motor tiltMotor;

  private final double startAngle = 90.851;
  private final double stowAngle = 75;
  private final double placeL4Angle = 55;
  private final double placeL3Angle = 60;
  private final double placeL2Angle = 73.15;
  private final double placeL1Angle = 33.37915;
  private final double handoffAngle = -85;
  private final double stationAngle = 70;
  private final double groundAngle = -30;
  private final double dislodgeAngle = 40;
  private final double algaeGroundAngle = -20;

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
              motor.getConfigurator().apply(config);
            },
            sim -> {
              sim.withHardstops(handoffAngle, startAngle);
            },
            90,
            FeedbackController.fromPID(
                new PIDController(0.04, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(2);
                }),
            FeedforwardController.forArmGravity(0.35, 0.034937, 0.015511, 0.0042897),
            TargetType.Position);

    tiltMotor.useThroughBoreEncoder(ArmWiring.ENCODER_CHANNEL, true, 0.81);
    tiltMotor.getSysIDCommands("Arm", 0.25, 0.5, 4).putOnDashboard("Arm", this);
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

  public Command algaeGround() {
    return Commands.runOnce(
            () -> {
              tiltMotor.setTarget(algaeGroundAngle);
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
              tiltMotor.setTarget(placeL4Angle);
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
