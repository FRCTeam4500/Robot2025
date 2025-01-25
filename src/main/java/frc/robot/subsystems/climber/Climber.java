package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

public class Climber extends SubsystemBase implements Loggable {
  private Motor extendMotor;
  private Motor tiltyMotor;
  public final MechanismLigament2d mech;
  private MechanismLigament2d inverseExtension;

  public Climber() {
    extendMotor =
        Motor.fromIdealSim(
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                pid -> {
                  pid.setTolerance(0.05);
                }),
            TargetType.Position,
            0.1);

    tiltyMotor =
        Motor.fromIdealSim(
            FeedbackController.fromPID(
                new PIDController(0, 0, 0),
                pid -> {
                  pid.setTolerance(1);
                }),
            TargetType.Position,
            0);
    mech = new MechanismLigament2d("Climber Tilt", .5, 0);
    inverseExtension = new MechanismLigament2d("Climber Inverse Extension", 0.4, 180);
    MechanismLigament2d latch = new MechanismLigament2d("Climber Latch", 0.2, -90);
    inverseExtension.append(latch);
    mech.append(inverseExtension);
  }

  public Command up() {
    return Commands.runOnce(
            () -> {
              extendMotor.setTarget(.5);
              tiltyMotor.setTarget(90);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return extendMotor.atTarget() && tiltyMotor.atTarget();
                }));
  }

  public Command down() {
    return Commands.runOnce(
            () -> {
              extendMotor.setTarget(.1);
              tiltyMotor.setTarget(0);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return extendMotor.atTarget() && tiltyMotor.atTarget();
                }));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "extend Motor", extendMotor);
    HoundLog.log(path, "tilty Motor", tiltyMotor);
  }

  @Override
  public void periodic() {
    mech.setAngle(tiltyMotor.getPosition());
    inverseExtension.setLength(.5 - extendMotor.getPosition());
  }
}
