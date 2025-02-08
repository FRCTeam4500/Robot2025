package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ScoringLocations {
  public static final Pose2d A = new Pose2d(3.0, 4.13, Rotation2d.fromDegrees(0));
  public static final Pose2d B = new Pose2d(2.94, 3.85, Rotation2d.fromDegrees(0));
  public static final Pose2d C = new Pose2d(3.54, 2.79, Rotation2d.fromDegrees(60));
  public static final Pose2d D = new Pose2d(3.82, 2.59, Rotation2d.fromDegrees(60));
  public static final Pose2d E = new Pose2d(5.19, 2.66, Rotation2d.fromDegrees(120));
  public static final Pose2d F = new Pose2d(5.47, 2.77, Rotation2d.fromDegrees(120));
  public static final Pose2d G = new Pose2d(5.99, 3.88, Rotation2d.fromDegrees(180));
  public static final Pose2d H = new Pose2d(5.99, 4.22, Rotation2d.fromDegrees(180));
  public static final Pose2d I = new Pose2d(5.32, 5.31, Rotation2d.fromDegrees(240));
  public static final Pose2d J = new Pose2d(5.11, 5.44, Rotation2d.fromDegrees(240));
  public static final Pose2d K = new Pose2d(3.81, 5.36, Rotation2d.fromDegrees(300));
  public static final Pose2d L = new Pose2d(3.61, 5.21, Rotation2d.fromDegrees(300));

  public static Pose2d allianceFlip(Pose2d blue) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
      return blue;
    }
    return flip(blue);
  }

  public static Pose2d flip(Pose2d og) {
    return new Pose2d(
        17.55 - og.getX(),
        8.05 - og.getY(),
        new Rotation2d(og.getRotation().getRadians() - Math.PI));
  }
}
