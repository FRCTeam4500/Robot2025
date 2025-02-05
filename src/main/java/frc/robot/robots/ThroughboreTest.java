package frc.robot.robots;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.WiringConstants.ArmWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.logging.HoundLog;

public class ThroughboreTest extends LoggedRobot {
  private Motor armMotor;
  // private DutyCycleEncoder encoder;
  private double voltage;

  public ThroughboreTest() {
    // encoder = new DutyCycleEncoder(0);
    // encoder.setInverted(true);
    armMotor = Motor.fromTalonFX(
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
        },
        90,
        FeedbackController.fromPID(
            new PIDController(0.1, 0, 0),
            (PIDController pid) -> {
              pid.setTolerance(2);
            }),
        FeedforwardController.forArmGravity(0.35, 0.11, 0, 0),
        // FeedforwardController.forNone(),
        TargetType.Position);
    armMotor.useThroughBoreEncoder(0, true, 0.81);
    Sendable sendable = new Sendable() {

      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Voltage", () -> {
          return voltage;
        }, (nextVoltage) -> voltage = nextVoltage);
      }

    };
    SmartDashboard.putData("Voltage Setter", sendable);
    SmartDashboard.putData("Target: 90", Commands.runOnce(() -> armMotor.setTarget(90)));
    SmartDashboard.putData("Target: 70", Commands.runOnce(() -> armMotor.setTarget(70)));
    SmartDashboard.putData("Target: 45", Commands.runOnce(() -> armMotor.setTarget(45)));
    SmartDashboard.putData("Target: 30", Commands.runOnce(() -> armMotor.setTarget(30)));
    SmartDashboard.putData("Target: 0", Commands.runOnce(() -> armMotor.setTarget(0)));
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("Motor", armMotor);
    CommandScheduler.getInstance().run();
    // HoundLog.log("Encoder stat", encoder.get());
  }
}

//cool :)