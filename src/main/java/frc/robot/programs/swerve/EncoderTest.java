package frc.robot.programs.swerve;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.WiringConstants;
import frc.robot.programs.LoggedRobot;
import frc.robot.utilities.logging.HoundLog;

public class EncoderTest extends LoggedRobot {
  private AnalogEncoder fl;
  private AnalogEncoder fr;
  private AnalogEncoder bl;
  private AnalogEncoder br;
  private DutyCycleEncoder armEncoder;
  private DutyCycleEncoder climberEncoder;
  private DigitalInput elevatorSwitch;

  public EncoderTest() {
    fl = new AnalogEncoder(0);
    fr = new AnalogEncoder(1);
    bl = new AnalogEncoder(2);
    br = new AnalogEncoder(3);
    armEncoder = new DutyCycleEncoder(WiringConstants.ArmWiring.ENCODER_CHANNEL);
    climberEncoder = new DutyCycleEncoder(WiringConstants.ClimberWiring.ENCODER_CHANNEL);
    elevatorSwitch = new DigitalInput(3);
    armEncoder.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("fl angle encoder", fl.get());
    HoundLog.log("fr angle encoder", fr.get());
    HoundLog.log("bl angle encoder", bl.get());
    HoundLog.log("br angle encoder", br.get());
    HoundLog.log("Arm Encoder", (armEncoder.get() - 0.288) * 360);
    HoundLog.log("Climber Encoder", 360 * (climberEncoder.get() + (0.25 - 0.177)));
    HoundLog.log("Elevator Switch", elevatorSwitch.get());
  }
}
