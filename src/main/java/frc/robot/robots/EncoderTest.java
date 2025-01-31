package frc.robot.robots;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.utilities.logging.HoundLog;;

public class EncoderTest extends LoggedRobot {
  private AnalogEncoder fl;
  private AnalogEncoder fr;
  private AnalogEncoder bl;
  private AnalogEncoder br;

  public EncoderTest() {
    fl = new AnalogEncoder(0);
    fr = new AnalogEncoder(1);
    bl = new AnalogEncoder(2);
    br = new AnalogEncoder(3);
  }

  @Override
  public void robotPeriodic() {
    HoundLog.log("fl angle encoder", fl.get());
    HoundLog.log("fr angle encoder", fr.get());
    HoundLog.log("bl angle encoder", bl.get());
    HoundLog.log("br angle encoder", br.get());
  }

}
