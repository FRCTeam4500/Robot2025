package frc.robot.programs.placer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.WiringConstants.PlacerWiring;
import frc.robot.hardware.Motor;
import frc.robot.hardware.Motor.TargetType;
import frc.robot.programs.LoggedRobot;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.FeedforwardController;
import frc.robot.utilities.FeedforwardSim;
import frc.robot.utilities.logging.HoundLog;

public class PlacerTest extends LoggedRobot {
  private Motor motor;

  public PlacerTest() {
    motor =
        Motor.fromTalonFX(
            PlacerWiring.PLACER_ID,
            (TalonFX fx) -> {
              TalonFXConfiguration config = new TalonFXConfiguration();
              config.Audio.AllowMusicDurDisable = true;
              config.CurrentLimits.StatorCurrentLimitEnable = false;
              config.CurrentLimits.SupplyCurrentLimitEnable = false;
              config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
              config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
              fx.getConfigurator().apply(config);
            },
            (FeedforwardSim sim) -> {},
            0,
            FeedbackController.empty(0.5),
            FeedforwardController.forConstantGravity(0, 0.47622, 0.12973, 0.01321),
            TargetType.Velocity);
    // SysIDCommands commands = motor.getSysIDCommands("Placer", 2, 8, 5);
    // SmartDashboard.putData("Placer Dynamic Forward", commands.dynamicForward());
    // SmartDashboard.putData("Placer Dynamic Reverse", commands.dynamicReverse());
    // SmartDashboard.putData("Placer Quasistatic Forward", commands.quasistaticForward());
    // SmartDashboard.putData("Placer Quasistatic Reverse", commands.quasistaticReverse());
    SmartDashboard.putData("Speed: 50", Commands.runOnce(() -> motor.setTarget(50)));
    SmartDashboard.putData("Speed: 25", Commands.runOnce(() -> motor.setTarget(25)));
    SmartDashboard.putData("Speed: 0", Commands.runOnce(() -> motor.setTarget(0)));
    SmartDashboard.putData("Speed: -25", Commands.runOnce(() -> motor.setTarget(-25)));
    SmartDashboard.putData("Speed: -50", Commands.runOnce(() -> motor.setTarget(-50)));
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("Motor", motor);
    CommandScheduler.getInstance().run();
  }
}
