package frc.robot;

/**
 * A class that holds wiring details for the robot. These include CAN IDs for motors, as well as
 * ports for the RIO. Each subsystem has its own subclass with subsystem specific information
 */
public class WiringConstants {
  /**
   * A class containing the wiring details for the swerve subsystem. Holds the 8 CAN IDs, as well as
   * DIO ports for the absolute encoders
   */
  public static class SwerveWiring {
    public static final int FRONT_LEFT_DRIVE_ID = 11;
    public static final int FRONT_LEFT_ANGLE_ID = 9;
    public static final int FRONT_RIGHT_DRIVE_ID = 10;
    public static final int FRONT_RIGHT_ANGLE_ID = 8;
    public static final int BACK_LEFT_DRIVE_ID = 19;
    public static final int BACK_LEFT_ANGLE_ID = 18;
    public static final int BACK_RIGHT_DRIVE_ID = 7;
    public static final int BACK_RIGHT_ANGLE_ID = 6;

    public static final int FRONT_LEFT_ENCODER_ID = 0;
    public static final int FRONT_RIGHT_ENCODER_ID = 1;
    public static final int BACK_LEFT_ENCODER_ID = 2;
    public static final int BACK_RIGHT_ENCODER_ID = 3;
  }

  public static class PlacerWiring {
    public static final int PLACER_ID = 12;
  }

  public static class ArmWiring {
    public static final int ARM_ID = 13;
    public static final int ENCODER_CHANNEL = 1;
  }

  public static class ElevatorWiring {
    public static final int ELEVATOR_ID = 17;
    public static final int ZEROING_CHANNEL = 3;
  }

  public static class ClimberWiring {
    public static final int CLIMBER_ID = 30;
    public static final int ENCODER_CHANNEL = 2;
  }

  public static class RampWiring {
    public static final int RAMP_ID = 5;
  }
}
