package frc.robot.robots;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.WiringConstants.ArmWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.logging.HoundLog;

public class ArmTest extends LoggedRobot {
  private Motor motor;

  public ArmTest() {
    motor =
        Motor.fromTalonFX(
            ArmWiring.ARM_ID,
            motor -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.Audio.AllowMusicDurDisable = true;
              config.Feedback.SensorToMechanismRatio = (60 / 12.0) * (60.0 / 18) / 360;
              config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
              config.CurrentLimits.StatorCurrentLimit = 60;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              motor.getConfigurator().apply(config);
            },
            sim -> {},
            90,
            FeedbackController.fromPID(
                new PIDController(0.005, 0.005, 0),
                (PIDController pid) -> {
                  pid.setTolerance(2);
                }),
            // FeedforwardController.forArmGravity(1.016, 0.297, 0.0046134, 0.00075716),
            FeedforwardController.forNone(),
            TargetType.Position);
    SmartDashboard.putData("Target: 90", Commands.runOnce(() -> motor.setTarget(90)));
    SmartDashboard.putData("Target: 0", Commands.runOnce(() -> motor.setTarget(0)));
    SmartDashboard.putData("Target: 45", Commands.runOnce(() -> motor.setTarget(45)));
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("Motor", motor);
    CommandScheduler.getInstance().run();
  }
}
