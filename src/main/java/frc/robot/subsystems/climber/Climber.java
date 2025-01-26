package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
  private Motor tiltyMotor;
  public final MechanismLigament2d mech;

  private final double climbAngle = 45;
  private final double readyAngle = 180;
  private final double stowAngle = 0;

  public Climber() {
    tiltyMotor =
        Motor.fromIdealSim(
            FeedbackController.fromProfiledPID(
              new ProfiledPIDController(0, 0, 0, new Constraints(90, 180)), 
              (ProfiledPIDController pid) -> {
                pid.setTolerance(1);
              }),
            TargetType.Meters,
            stowAngle);
    mech = new MechanismLigament2d("Climber", .3, stowAngle);
  }

  public Command stow() {
    return Commands.runOnce(
            () -> {
              tiltyMotor.setTarget(stowAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltyMotor.atTarget();
                }));
  }

  public Command ready() {
    return Commands.runOnce(
            () -> {
              tiltyMotor.setTarget(readyAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltyMotor.atTarget();
                }));
  }

  public Command climb() {
    return Commands.runOnce(
            () -> {
              tiltyMotor.setTarget(climbAngle);
            },
            this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return tiltyMotor.atTarget();
                }));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Tilty Motor", tiltyMotor);
    mech.setAngle(tiltyMotor.getPosition());
  }
}
