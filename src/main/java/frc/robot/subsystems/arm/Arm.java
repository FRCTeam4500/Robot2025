package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Arm extends SubsystemBase implements Loggable {
  private Motor tiltMotor;
  public final MechanismLigament2d mech;

  private final double startAngle = 90.851;
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
    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    map.put(-6.81, 0.15);
    map.put(21.2, 0.30);
    map.put(49.77, 0.41);
    map.put(51.67, 0.48);
    map.put(81.0, 0.26);
    map.put(10.0, 0.23);
    map.put(0.0, 0.16);
    map.put(30.0, 0.34);
    map.put(60.0, 0.6);
    map.put(70.0, 0.48);
    map.put(65.0, 0.53);
    map.put(55.0, 0.64);
    map.put(-10.0, 0.03);
    map.put(-20.0, -0.05);
    map.put(-30.0, -0.1);
    map.put(-40.0, -0.18);
    map.put(-50.0, -0.31);
    map.put(-60.0, -0.34);
    map.put(-70.0, -0.38);

    tiltMotor =
        Motor.fromTalonFX(
            ArmWiring.ARM_ID,
            motor -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.Audio.AllowMusicDurDisable = true;
              config.Feedback.SensorToMechanismRatio = (60 / 12.0) * (60.0 / 18) / 360;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.CurrentLimits.StatorCurrentLimit = 60;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              motor.getConfigurator().apply(config);
            },
            sim -> {},
            90,
            FeedbackController.fromPID(
                new PIDController(0.03, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(2);
                }),
            // FeedforwardController.forArmGravity(1.016, 0.297, 0.0046134, 0.00075716),
            FeedforwardController.forGasShockArm(map, 0, 0, 0),
            TargetType.Position);
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
