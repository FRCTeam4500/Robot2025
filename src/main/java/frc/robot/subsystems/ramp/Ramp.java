package frc.robot.subsystems.ramp;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

  private Alert configError = new Alert("Ramp Config Failed :(", AlertType.kError);

  private double hideAngle = -390;

  /** Creates a new Ramp subsystem. */
  public Ramp() {
    tiltMotor =
        Motor.fromSparkMax(
            RampWiring.RAMP_ID,
            false,
            (SparkMax max) -> {
              SparkMaxConfig config = new SparkMaxConfig();
              config.idleMode(IdleMode.kBrake);
              config.inverted(false);
              config.encoder.positionConversionFactor((1.0 / (60 / 12)) * 360);
              config.encoder.velocityConversionFactor((1.0 / (60 / 12) * 360));
              config.smartCurrentLimit(60);
              REVLibError err =
                  max.configure(
                      config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
              if (!err.equals(REVLibError.kOk)) configError.set(true);
              else configError.set(false);
            },
            (FeedforwardSim jim) -> {
              jim.withHardstops(90, 270);
            },
            -192,
            FeedbackController.fromPID(
                new PIDController(0.03, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(5);
                }),
            FeedforwardController.forNone(),
            TargetType.Position);
    tiltMotor.getSysIDCommands("Ramp", 0.2, 0.5, 4).putOnDashboard("Ramp", this);
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
    return Commands.runOnce(
            () -> {
              moveRamp(hideAngle);
            })
        .andThen(Commands.waitUntil(() -> tiltMotor.getPosition() < hideAngle))
        .andThen(Commands.runOnce(() -> tiltMotor.setVoltage(0), this));
  }

  /**
   * Moves the ramp to the intake angle.
   *
   * @param Command to move ramp to the intake angle.
   */
  public Command show() {
    return Commands.runOnce(() -> tiltMotor.setTarget(0))
        .andThen(Commands.runOnce(() -> tiltMotor.setVoltage(0), this));
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Motor", tiltMotor);
  }

  public double getAngle() {
    return tiltMotor.getPosition();
  }
}
