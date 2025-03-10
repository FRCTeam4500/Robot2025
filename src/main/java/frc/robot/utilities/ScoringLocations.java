package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.Swerve.Alignment;

public class ScoringLocations {
  private static Pose2d blueA = new Pose2d(2.99, 4.19, Rotation2d.fromDegrees(0));
  private static Pose2d blueB = new Pose2d(2.99, 3.86, Rotation2d.fromDegrees(0));
  private static Translation2d blueCenter = new Translation2d(4.48, 4.03);

  private static Pose2d redA = new Pose2d(14.56, 3.86, Rotation2d.fromDegrees(180));
  private static Pose2d redB = new Pose2d(14.56, 4.19, Rotation2d.fromDegrees(180));
  private static Translation2d redCenter = new Translation2d(13.07, 4.025);

  public static Pose2d A;
  public static Pose2d B;
  public static Translation2d center;

  static {
    Trigger isBlue =
        new Trigger(() -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue));
    isBlue.onTrue(
        Commands.runOnce(
                () -> {
                  center = blueCenter;
                  A = blueA;
                  B = blueB;
                  System.out.println(center);
                })
            .ignoringDisable(true));
    isBlue.onFalse(
        Commands.runOnce(
                () -> {
                  center = redCenter;
                  A = redA;
                  B = redB;
                  System.out.println(center);
                })
            .ignoringDisable(true));
    if (isBlue.getAsBoolean()) {
      center = blueCenter;
      A = blueA;
      B = blueB;
    } else {
      center = redCenter;
      A = redA;
      B = redB;
    }
  }

  public static Rotation2d getRotation(int id) {
    switch (id) {
      case 19:
        return Rotation2d.fromDegrees(60);
      case 18:
        return Rotation2d.fromDegrees(0);
      case 17:
        return Rotation2d.fromDegrees(-60);
      case 22:
        return Rotation2d.fromDegrees(-120);
      case 21:
        return Rotation2d.fromDegrees(180);
      case 20:
        return Rotation2d.fromDegrees(120);
      case 9:
        return Rotation2d.fromDegrees(60);
      case 10:
        return Rotation2d.fromDegrees(0);
      case 11:
        return Rotation2d.fromDegrees(-60);
      case 6:
        return Rotation2d.fromDegrees(-120);
      case 7:
        return Rotation2d.fromDegrees(180);
      case 8:
        return Rotation2d.fromDegrees(120);
      default:
        return Rotation2d.fromDegrees(0);
    }
  }

  public static boolean isReef(int id) {
    return (id >= 6 && id <= 11) || (id >= 17 && id <= 22);
  }

  public static void setupBlue(Pose2d A, Pose2d B, Pose2d AB, Pose2d GH) {
    blueA = A;
    blueB = B;
    blueCenter = AB.getTranslation().interpolate(GH.getTranslation(), 0.5);
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
      ScoringLocations.center = blueCenter;
      ScoringLocations.A = blueA;
      ScoringLocations.B = blueB;
    }
  }

  public static void setupRed(Pose2d A, Pose2d B, Pose2d AB, Pose2d GH) {
    redA = A;
    redB = B;
    redCenter = AB.getTranslation().interpolate(GH.getTranslation(), 0.5);
    if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
      ScoringLocations.center = redCenter;
      ScoringLocations.A = redA;
      ScoringLocations.B = redB;
    }
  }

  public static int getDriveTag(Translation2d robotPose) {
    double angle = robotPose.minus(center).getAngle().getDegrees();
    angle = MathUtil.inputModulus(angle, -30, 330);
    switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
      case Blue:
        if (angle <= 30) {
          return 21;
        } else if (angle <= 90) {
          return 20;
        } else if (angle <= 150) {
          return 19;
        } else if (angle <= 210) {
          return 18;
        } else if (angle <= 270) {
          return 17;
        } else {
          return 22;
        }
      default:
        if (angle <= 30) {
          return 7;
        } else if (angle <= 90) {
          return 8;
        } else if (angle <= 150) {
          return 9;
        } else if (angle <= 210) {
          return 10;
        } else if (angle <= 270) {
          return 11;
        } else {
          return 6;
        }
    }
  }

  public static Pose2d getDriveTarget(Translation2d robotPose, Alignment alignment) {
    double angle = robotPose.minus(center).getAngle().getDegrees();
    angle = MathUtil.inputModulus(angle, -30, 330);
    switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
      case Red:
        if (angle <= 30) {
          switch (alignment) {
            case Left:
              return getA();
            case Right:
              return getB();
            case Top:
            case Middle:
            case Bottom:
              return getA().interpolate(getB(), 0.5);
          }
        } else if (angle <= 90) {
          switch (alignment) {
            case Left:
            case Bottom:
              return getC();
            case Right:
            case Top:
              return getD();
            case Middle:
              return getC().interpolate(getD(), 0.5);
          }
        } else if (angle <= 150) {
          switch (alignment) {
            case Left:
            case Bottom:
              return getE();
            case Right:
            case Top:
              return getF();
            case Middle:
              return getE().interpolate(getF(), 0.5);
          }
        } else if (angle <= 210) {
          switch (alignment) {
            case Left:
              return getG();
            case Right:
              return getH();
            case Top:
            case Middle:
            case Bottom:
              return getG().interpolate(getH(), 0.5);
          }
        } else if (angle <= 270) {
          switch (alignment) {
            case Left:
            case Top:
              return getI();
            case Right:
            case Bottom:
              return getJ();
            case Middle:
              return getI().interpolate(getJ(), 0.5);
          }
        } else {
          switch (alignment) {
            case Left:
            case Top:
              return getK();
            case Right:
            case Bottom:
              return getL();
            case Middle:
              return getK().interpolate(getL(), 0.5);
          }
        }
      default:
        if (angle <= 30) {
          switch (alignment) {
            case Left:
              return getG();
            case Right:
              return getH();
            case Top:
            case Middle:
            case Bottom:
              return getG().interpolate(getH(), 0.5);
          }
        } else if (angle <= 90) {
          switch (alignment) {
            case Left:
            case Top:
              return getI();
            case Right:
            case Bottom:
              return getJ();
            case Middle:
              return getI().interpolate(getJ(), 0.5);
          }
        } else if (angle <= 150) {
          switch (alignment) {
            case Left:
            case Top:
              return getK();
            case Right:
            case Bottom:
              return getL();
            case Middle:
              return getK().interpolate(getL(), 0.5);
          }
        } else if (angle <= 210) {
          switch (alignment) {
            case Left:
              return getA();
            case Right:
              return getB();
            case Top:
            case Middle:
            case Bottom:
              return getA().interpolate(getB(), 0.5);
          }
        } else if (angle <= 270) {
          switch (alignment) {
            case Left:
            case Bottom:
              return getC();
            case Right:
            case Top:
              return getD();
            case Middle:
              return getC().interpolate(getD(), 0.5);
          }
        } else {
          switch (alignment) {
            case Left:
            case Bottom:
              return getE();
            case Right:
            case Top:
              return getF();
            case Middle:
              return getE().interpolate(getF(), 0.5);
          }
        }
    }
    return getA();
  }

  // public static final Pose2d A = new Pose2d(2.99, 4.19, Rotation2d.fromDegrees(0));
  // public static final Pose2d B = new Pose2d(2.99, 3.86, Rotation2d.fromDegrees(0));
  // public static final Pose2d C = new Pose2d(3.60, 2.79, Rotation2d.fromDegrees(60));
  // public static final Pose2d D = new Pose2d(3.89, 2.63, Rotation2d.fromDegrees(60));
  // public static final Pose2d E = new Pose2d(5.09, 2.65, Rotation2d.fromDegrees(120));
  // public static final Pose2d F = new Pose2d(5.37, 2.83, Rotation2d.fromDegrees(120));
  // public static final Pose2d G = new Pose2d(5.97, 3.86, Rotation2d.fromDegrees(180));
  // public static final Pose2d H = new Pose2d(5.97, 4.19, Rotation2d.fromDegrees(180));
  // public static final Pose2d I = new Pose2d(5.37, 5.22, Rotation2d.fromDegrees(240));
  // public static final Pose2d J = new Pose2d(5.09, 5.38, Rotation2d.fromDegrees(240));
  // public static final Pose2d K = new Pose2d(3.89, 5.38, Rotation2d.fromDegrees(300));
  // public static final Pose2d L = new Pose2d(3.60, 5.22, Rotation2d.fromDegrees(300));

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

  public static Pose2d getA() {
    return A;
  }

  public static Pose2d getB() {
    return B;
  }

  public static Pose2d getC() {
    return A.rotateAround(center, Rotation2d.fromDegrees(60));
  }

  public static Pose2d getD() {
    return B.rotateAround(center, Rotation2d.fromDegrees(60));
  }

  public static Pose2d getE() {
    return A.rotateAround(center, Rotation2d.fromDegrees(120));
  }

  public static Pose2d getF() {
    return B.rotateAround(center, Rotation2d.fromDegrees(120));
  }

  public static Pose2d getG() {
    return A.rotateAround(center, Rotation2d.fromDegrees(180));
  }

  public static Pose2d getH() {
    return B.rotateAround(center, Rotation2d.fromDegrees(180));
  }

  public static Pose2d getI() {
    return A.rotateAround(center, Rotation2d.fromDegrees(240));
  }

  public static Pose2d getJ() {
    return B.rotateAround(center, Rotation2d.fromDegrees(240));
  }

  public static Pose2d getK() {
    return A.rotateAround(center, Rotation2d.fromDegrees(300));
  }

  public static Pose2d getL() {
    return B.rotateAround(center, Rotation2d.fromDegrees(300));
  }
}
