package frc.robot.programs.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.WiringConstants.ArmWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.programs.LoggedRobot;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;

public class ArmTest extends LoggedRobot {
  private Motor armMotor;

  public ArmTest() {
    armMotor =
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
              sim.withHardstops(-90, 90);
            },
            90,
            FeedbackController.fromPID(
                new PIDController(0.08, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(2);
                }),
            FeedforwardController.forArmGravity(0.35, 0.034937, 0.015511, 0.0042897),
            TargetType.Position);
    armMotor.useThroughBoreEncoder(ArmWiring.ENCODER_CHANNEL, true, 0.81);
    SysIDCommands commands = armMotor.getSysIDCommands("Arm", 0.25, 0.5, 4);
    SmartDashboard.putData("Arm Dynamic Forward", commands.dynamicForward());
    SmartDashboard.putData("Arm Dynamic Reverse", commands.dynamicReverse());
    SmartDashboard.putData("Arm Quasistatic Forward", commands.quasistaticForward());
    SmartDashboard.putData("Arm Quasistatic Reverse", commands.quasistaticReverse());
    // SmartDashboard.putData("Target: 70", Commands.runOnce(() -> armMotor.setTarget(70)));
    // SmartDashboard.putData("Target: 0", Commands.runOnce(() -> armMotor.setTarget(0)));
    // SmartDashboard.putData("Target: 45", Commands.runOnce(() -> armMotor.setTarget(45)));
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("Motor", armMotor);
    CommandScheduler.getInstance().run();
  }
}
