package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.utilities.logging.HoundLog;
import java.util.function.Supplier;

public class StopTilting {
  private static ChassisSpeeds previousSpeed = new ChassisSpeeds();
  private static SwerveDriveKinematics kinematics;
  private static Supplier<Pose2d> pose;

  private static double forward;
  private static double backward;
  private static double left;
  private static double right;

  private static final Transform3d BASE_COM = new Transform3d(0, 0, 0.1, Rotation3d.kZero);
  private static final Transform3d ARM_COM = new Transform3d(0.25, 0, 0, Rotation3d.kZero);
  private static final Transform3d CARRIAGE_COM = new Transform3d(0, 0, 0.1, Rotation3d.kZero);
  private static final Transform3d ELEVATOR_COM = new Transform3d(0, 0, 0.4, Rotation3d.kZero);
  private static final Transform3d CLIMBER_COM = new Transform3d(0.2, 0, 0, Rotation3d.kZero);
  private static final Transform3d RAMP_COM = new Transform3d(0.1, 0, 0, Rotation3d.kZero);

  private static final double BASE_MASS = 39.3468644;
  private static final double ELEVATOR_MASS = 2.5197056;
  private static final double CARRIAGE_MASS = 1.81437;
  private static final double ARM_MASS = 5.44311;
  private static final double CLIMBER_MASS = 3.17515;
  private static final double RAMP_MASS = 2.5;

  private static final double g = 9.81;

  public static void setupKinematics(SwerveDriveKinematics kinematics) {
    StopTilting.kinematics = kinematics;
  }

  public static void setupPose(Supplier<Pose2d> pose) {
    StopTilting.pose = pose;
  }

  public static void updateCenterOfMass(
      Transform3d armPose,
      Transform3d elevatorPose,
      Transform3d carriagePose,
      Transform3d climberPose,
      Transform3d rampPose) {
    Pose3d robot = new Pose3d(pose.get());
    HoundLog.log("COM", "Arm", robot.plus(armPose.plus(ARM_COM)));
    HoundLog.log("COM", "Elevator Stage", robot.plus(elevatorPose.plus(ELEVATOR_COM)));
    HoundLog.log("COM", "Carriage", robot.plus(carriagePose.plus(CARRIAGE_COM)));
    HoundLog.log("COM", "Climber", robot.plus(climberPose.plus(CLIMBER_COM)));
    HoundLog.log("COM", "Ramp", robot.plus(rampPose.plus(RAMP_COM)));
    Transform3d centerOfMass =
        new Transform3d(
            (armPose.plus(ARM_COM).getX() * ARM_MASS
                    + elevatorPose.plus(ELEVATOR_COM).getX() * ELEVATOR_MASS
                    + carriagePose.plus(CARRIAGE_COM).getX() * CARRIAGE_MASS
                    + climberPose.plus(CLIMBER_COM).getX() * CLIMBER_MASS
                    + rampPose.plus(RAMP_COM).getX() * RAMP_MASS
                    + BASE_COM.getX() * BASE_MASS)
                / (BASE_MASS + ELEVATOR_MASS + CARRIAGE_MASS + ARM_MASS + CLIMBER_MASS),
            (armPose.plus(ARM_COM).getY() * ARM_MASS
                    + elevatorPose.plus(ELEVATOR_COM).getY() * ELEVATOR_MASS
                    + carriagePose.plus(CARRIAGE_COM).getY() * CARRIAGE_MASS
                    + climberPose.plus(CLIMBER_COM).getY() * CLIMBER_MASS
                    + rampPose.plus(RAMP_COM).getY() * RAMP_MASS
                    + BASE_COM.getY() * BASE_MASS)
                / (BASE_MASS + ELEVATOR_MASS + CARRIAGE_MASS + ARM_MASS + CLIMBER_MASS),
            (armPose.plus(ARM_COM).getZ() * ARM_MASS
                    + elevatorPose.plus(ELEVATOR_COM).getZ() * ELEVATOR_MASS
                    + carriagePose.plus(CARRIAGE_COM).getZ() * CARRIAGE_MASS
                    + climberPose.plus(CLIMBER_COM).getZ() * CLIMBER_MASS
                    + rampPose.plus(RAMP_COM).getZ() * RAMP_MASS
                    + BASE_COM.getZ() * BASE_MASS)
                / (BASE_MASS + ELEVATOR_MASS + CARRIAGE_MASS + ARM_MASS + CLIMBER_MASS),
            Rotation3d.kZero);
    HoundLog.log("COM", "COM", robot.plus(centerOfMass));
    double number = 0;
    for (Translation2d tl : kinematics.getModules()) {
      if (tl.getX() > number) {
        number = tl.getX();
      }
    }
    forward = (g / centerOfMass.getZ()) * (number - centerOfMass.getX());
    number = 0;
    for (Translation2d tl : kinematics.getModules()) {
      if (tl.getX() < number) {
        number = tl.getX();
      }
    }
    backward = (g / centerOfMass.getZ()) * (number - centerOfMass.getX());
    number = 0;
    for (Translation2d tl : kinematics.getModules()) {
      if (tl.getY() > number) {
        number = tl.getY();
      }
    }
    left = (g / centerOfMass.getZ()) * (number - centerOfMass.getY());
    number = 0;
    for (Translation2d tl : kinematics.getModules()) {
      if (tl.getY() < number) {
        number = tl.getY();
      }
    }
    right = (g / centerOfMass.getZ()) * (number - centerOfMass.getY());
    HoundLog.log("COM", "Max Forward Accel", forward);
    HoundLog.log("COM", "Max Backwards Accel", backward);
    HoundLog.log("COM", "Max Left Accel", left);
    HoundLog.log("COM", "Max Right Accel", right);
  }

  public static ChassisSpeeds limitAccel(ChassisSpeeds target) {
    ChassisSpeeds accel =
        new ChassisSpeeds(
            (target.vxMetersPerSecond - previousSpeed.vxMetersPerSecond) / 0.02,
            (target.vyMetersPerSecond - previousSpeed.vyMetersPerSecond) / 0.02,
            (target.omegaRadiansPerSecond - previousSpeed.omegaRadiansPerSecond) / 0.02);
    double forwardAccel = 0;
    double sidewaysAccel = 0;
    Translation2d accel2d = new Translation2d(accel.vxMetersPerSecond, accel.vyMetersPerSecond);
    if (accel2d.getX() > 0) {
      forwardAccel = Math.min(forward, accel2d.getX());
    } else {
      forwardAccel = Math.max(backward, accel2d.getX());
    }
    if (accel2d.getY() > 0) {
      sidewaysAccel = Math.min(left, accel2d.getY());
    } else {
      sidewaysAccel = Math.max(right, accel2d.getY());
    }
    previousSpeed =
        new ChassisSpeeds(
            forwardAccel * 0.02 + previousSpeed.vxMetersPerSecond,
            sidewaysAccel * 0.02 + previousSpeed.vyMetersPerSecond,
            target.omegaRadiansPerSecond);
    return previousSpeed;
  }
}
