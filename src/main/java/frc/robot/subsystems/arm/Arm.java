package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.WiringConstants.ArmWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Arm extends SubsystemBase implements Loggable {
  private Motor tiltMotor;
  public final MechanismLigament2d mech;

  private final double startAngle = 101.851;
  private final double stowAngle = 75;
  private final double placeL4Angle = 70;
  private final double placeL3Angle = 60;
  private final double placeL2Angle = 60;
  private final double placeL1Angle = 60;
  private final double handoffAngle = -90;
  // private final double handoffAngle = -82.777;
  private final double stationAngle = 75;

  public final Trigger canMoveElevator =
      new Trigger(() -> tiltMotor.getPosition() > -25 && tiltMotor.getPosition() < 77);

  public Arm() {
    tiltMotor =
        Motor.fromTalonFX(
            ArmWiring.ARM_ID,
            (TalonFX fx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.CurrentLimits.StatorCurrentLimit = 60;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.Feedback.SensorToMechanismRatio = (60 / 12.0) * (60.0 / 18) / 360;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              fx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {
              sim.withHardstops(handoffAngle, startAngle);
            },
            startAngle,
            FeedbackController.fromPID(
                new PIDController(0.01, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(2);
                }),
            FeedforwardController.forArmGravity(1.016, 0.297, 0.0046134, 0.00075716),
            TargetType.Degrees);
    mech = new MechanismLigament2d("Arm", .5, startAngle);
    mech.append(new MechanismLigament2d("Placer", 0.1, -90));
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
    mech.setAngle(tiltMotor.getPosition());
  }
}
