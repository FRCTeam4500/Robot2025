package frc.robot.subsystems.ramp;

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
import frc.robot.WiringConstants.RampWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;

/** The Ramp subsystem is used by the robot to intake game pieces. */
public class Ramp extends SubsystemBase implements Loggable {
  private Motor tiltMotor;
  public final MechanismLigament2d mech;

  private double intakeAngle = -185;
  private double stowAngle = -250;

  /** Creates a new Ramp subsystem. */
  public Ramp() {
    tiltMotor =
        Motor.fromSparkMax(
            RampWiring.RAMP_ID,
            false,
            (SparkMax max) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.idleMode(IdleMode.kBrake);
              config.inverted(true);
              config.encoder.positionConversionFactor((1.0 / ((60 / 12) * (60 / 18))) * 360);
              config.encoder.velocityConversionFactor((1.0 / ((60 / 12) * (60 / 18))) * 360);
              config.smartCurrentLimit(60);
              max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            (FeedforwardSim jim) -> {
              jim.withHardstops(90, 270);
            },
            -170.713,
            FeedbackController.fromPID(
                new PIDController(0.015, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(5);
                }),
            FeedforwardController.forArmGravity(0.31, 0.07, 0, 0),
            TargetType.Position);
    mech = new MechanismLigament2d("Ramp", 0.4, intakeAngle);
  }

  /**
   * Moves the ramp to the specified target angle.
   *
   * @param targetAngle the target angle in rad.
   */
  private void moveRamp(double targetAngle) {
    tiltMotor.setTarget(targetAngle);
  }

  /**
   * Moves the ramp to the stow angle.
   *
   * @return Command to move ramp to the stow angle.
   */
  public Command hide() {
    return Commands.runOnce(() -> moveRamp(stowAngle));
  }

  /**
   * Moves the ramp to the intake angle.
   *
   * @param Command to move ramp to the intake angle.
   */
  public Command show() {
    return Commands.runOnce(() -> moveRamp(intakeAngle));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Motor", tiltMotor);
    mech.setAngle(tiltMotor.getPosition());
  }

  public double getAngle() {
    return tiltMotor.getPosition();
  }
}
