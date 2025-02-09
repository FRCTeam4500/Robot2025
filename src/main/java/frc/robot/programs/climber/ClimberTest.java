package frc.robot.programs.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.WiringConstants.ClimberWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.programs.LoggedRobot;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.SysIDCommands;
import frc.robot.utilities.logging.HoundLog;

public class ClimberTest extends LoggedRobot {
  private Motor climberMotor;

  public ClimberTest() {
    climberMotor =
        Motor.fromTalonFX(
            ClimberWiring.CLIMBER_ID,
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
                new PIDController(0.00, 0, 0),
                (PIDController pid) -> {
                  pid.setTolerance(2);
                }),
            null,
            TargetType.Position);

    SysIDCommands commands = climberMotor.getSysIDCommands("climber", 1, 1, 5);
    SmartDashboard.putData("climberQuasistaticForward", commands.quasistaticForward());
    SmartDashboard.putData("climberQuasistaticReverse", commands.quasistaticReverse());
    SmartDashboard.putData("climberDyanamicForward", commands.dynamicForward());
    SmartDashboard.putData("climberDynamicReverse", commands.dynamicReverse());
    // SmartDashboard.putData("climber to 10", Commands.runOnce(() -> climberMotor.setTarget(10)));
    // SmartDashboard.putData("climber to 0", Commands.runOnce(() -> climberMotor.setTarget(0)));
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("motor", climberMotor);
    CommandScheduler.getInstance().run();
  }
}
