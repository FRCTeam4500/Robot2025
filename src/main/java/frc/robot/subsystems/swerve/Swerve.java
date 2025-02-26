package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.utilities.ExtendedMath.withHardDeadzone;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.Gyro;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.PoseEstimate;
import frc.robot.utilities.FeedbackController;
import frc.robot.utilities.PoseFeedbackController;
import frc.robot.utilities.ScoringLocations;
import frc.robot.utilities.gamepieces.GamepieceManager;
import frc.robot.utilities.logging.HoundLog;
import frc.robot.utilities.logging.Loggable;
import java.util.Set;

/** The subsystem that controls our drivetrain, which is known as a swerve drive. */
public class Swerve extends SubsystemBase implements Loggable {
  private Gyro gyro;
  private SwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator estimator;
  private Limelight[] tagCameras;
  private Rotation2d targetHeading;
  private FeedbackController headingFeedback;
  private PoseFeedbackController poseFeedback;
  private Pose2d targetPose;

  public final Trigger closerToRight =
      new Trigger(
          () -> {
            switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
              case Blue:
                return estimator.getEstimatedPosition().getY() < 4;
              default:
                return estimator.getEstimatedPosition().getY() > 4;
            }
          });

  /** Creates a new {@link Swerve} using the constants defined in {@link SwerveConstants} */
  public Swerve() {
    tagCameras =
        new Limelight[] {new Limelight("limelight-hehehe"), new Limelight("limelight-hihihi")};
    if (RobotBase.isReal()) { // running on hardware robot
      gyro = Gyro.fromNavX(navx -> {});
    } else { // running robot simulation
      gyro = Gyro.fromSim(() -> getSpeeds().omegaRadiansPerSecond);
    }
    modules =
        new SwerveModule[] {
          FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE, BACK_LEFT_MODULE, BACK_RIGHT_MODULE
        };
    kinematics =
        new SwerveDriveKinematics(
            FRONT_LEFT_TRANSLATION,
            FRONT_RIGHT_TRANSLATION,
            BACK_LEFT_TRANSLATION,
            BACK_RIGHT_TRANSLATION);
    estimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getAngle(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.5, 0.5, 0.5),
            VecBuilder.fill(50, 50, 50));
    targetHeading = new Rotation2d();
    headingFeedback =
        FeedbackController.fromPID(
            5,
            0,
            0,
            pid -> {
              pid.enableContinuousInput(-Math.PI, Math.PI);
              pid.setTolerance(Math.PI / 32, Math.PI / 32);
              pid.setSetpoint(0);
            });

    poseFeedback =
        new PoseFeedbackController(
            FeedbackController.fromPID(
                5,
                0,
                0,
                pid -> {
                  pid.setTolerance(0.005);
                }),
            FeedbackController.fromPID(
                5,
                0,
                0,
                pid -> {
                  pid.setTolerance(0.005);
                }),
            FeedbackController.fromPID(
                6,
                0,
                0,
                pid -> {
                  pid.enableContinuousInput(0, 360);
                  pid.setTolerance(1);
                }));
    targetPose = new Pose2d();

    GamepieceManager.setRobotPoseSupplier(estimator::getEstimatedPosition);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      Alert alert = new Alert("READING AUTO CONFIG FILE FAILED!!", AlertType.kError);
      alert.set(true);
      alert.close();
      System.out.println(e.getMessage());
      config =
          new RobotConfig(
              68, // robot's mass in kg
              6.884, // Robot's moment of inertia
              new ModuleConfig(0.5, 6, 1.2, DCMotor.getKrakenX60(1).withReduction(5.143), 60, 1),
              FRONT_LEFT_TRANSLATION.getY()
                  * 2 // the trackwidth of the robot (dist. from top left to top right for example)
              );
    }
    AutoBuilder.configure(
        estimator::getEstimatedPosition,
        this::resetPose,
        this::getSpeeds,
        this::drive,
        new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(0.5)),
        config,
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          return alliance == Alliance.Red;
        },
        this);

    FRONT_LEFT_MODULE
        .getAngleMotor()
        .getSysIDCommands(
            "Swerve Angle",
            1,
            5,
            5,
            FRONT_RIGHT_MODULE.getAngleMotor(),
            BACK_LEFT_MODULE.getAngleMotor(),
            BACK_RIGHT_MODULE.getAngleMotor())
        .putOnDashboard("Swerve Angle", this);

    FRONT_LEFT_MODULE
        .getDriveMotor()
        .getSysIDCommands(
            "Swerve Drive",
            1,
            2.5,
            3,
            FRONT_RIGHT_MODULE.getDriveMotor(),
            BACK_LEFT_MODULE.getDriveMotor(),
            BACK_RIGHT_MODULE.getDriveMotor())
        .putOnDashboard("Swerve Drive", this);

    SmartDashboard.putData(
        "Characterization/Swerve/Face wheels forward", makePushable(Rotation2d.kZero));
    SmartDashboard.putData(
        "Characterization/Swerve/Conversion Factor Finder", driveConversionFinder(0.25, 5));
  }

  /**
   * @param xbox The {@link XboxController that will control the driving}
   * @return a {@link Command} that drives the robot using a {@link XboxController}.
   *     <ul>
   *       <li>Translation of the robot is controlled with the left stick, field relative
   *       <li>Rotation of the robot is controlled with the right stick
   *       <li>A target angle can be set using {@link #setTargetHeading}, which the robot will then
   *           turn to.
   *     </ul>
   */
  public Command angleCentric(XboxController xbox) {
    return Commands.run(
      () -> {
        drive(calculateVelRobotRel(xbox));
      },
      this)
      .beforeStarting(() -> targetHeading = estimator.getEstimatedPosition().getRotation())
      .withName("Angle Centric");
    }
    
    public Command robotCentric(XboxController xbox) {
      return Commands.run(
        () -> {
          double coefficient = Math.max(1 - xbox.getLeftTriggerAxis(), MIN_COEFFICIENT);
          drive(
            new ChassisSpeeds(
              coefficient
              * withHardDeadzone(-xbox.getLeftY(), 0.1)
              * MAX_SPEEDS.vxMetersPerSecond,
              coefficient
              * withHardDeadzone(-xbox.getLeftX(), 0.1)
              * MAX_SPEEDS.vyMetersPerSecond,
              coefficient
              * withHardDeadzone(-xbox.getRightX(), 0.1)
              * MAX_SPEEDS.omegaRadiansPerSecond));
            },
            this)
            .withName("Robot Centric");
          }
          
          public Command reefCentric(XboxController xbox) {
            return Commands.run(() -> {
              targetHeading = ScoringLocations.getDriveTarget(estimator.getEstimatedPosition().getTranslation(), Alignment.Middle).getRotation();
              drive(calculateVelRobotRel(xbox));
            }, this).withName("Reef Centric");
          }

          public Command poseCentric(Pose2d target) {
            return Commands.run(
              () -> {
                Pose2d current = estimator.getEstimatedPosition();
              ChassisSpeeds speeds = poseFeedback.calculate(current, target);
              drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, current.getRotation()));
            },
            this)
        .beforeStarting(
            () -> {
              poseFeedback.reset(estimator.getEstimatedPosition());
              this.targetPose = target;
            })
        .finallyDo(
            () -> {
              this.targetPose = new Pose2d();
            })
        .until(() -> poseFeedback.atTarget())
        .withName("Pose Centric");
  }

  public Command alignToReef(Alignment position) {
    return Commands.defer(
            () -> {
              return poseCentric(
                  ScoringLocations.getDriveTarget(
                      estimator.getEstimatedPosition().getTranslation(), position));
            },
            Set.of(this))
        .withName("Align To Reef: " + position.name() + " Side");
  }

  public Command turnToReef() {
    return Commands.defer(() -> {
      return setTargetHeading(ScoringLocations.getDriveTarget(
        estimator.getEstimatedPosition().getTranslation(), Alignment.Middle
      ).getRotation());
    }, Set.of(this));
  }


  /**
   * Updates the heading of the robot
   *
   * @param newHeading The robots new heading
   * @return A {@link Command} that can be bound to a {@link Trigger}
   */
  public Command resetHeading(Rotation2d newHeading) {
    return Commands.runOnce(
        () -> {
          resetPose(new Pose2d(estimator.getEstimatedPosition().getTranslation(), newHeading));
          targetHeading = newHeading;
        });
  }

  /**
   * Sets the target heading for the robot in tele-op only
   *
   * @param targetHeading The target heading
   * @return A {@link Command} that can be bound to a {@link Trigger}
   */
  public Command setTargetHeading(Rotation2d targetHeading) {
    return Commands.runOnce(() -> this.targetHeading = targetHeading);
  }

  public Command driveConversionFinder(double speed, double duration) {
    class CharacterizationState {
      double[] startPositions = new double[4];
      double[] endPositions = new double[4];
      double startAngle = 0;
      double endAngle = 0;
    }
    CharacterizationState state = new CharacterizationState();
    return Commands.runOnce(
            () -> {
              drive(new ChassisSpeeds(0, 0, speed));
            },
            this)
        .andThen(Commands.waitSeconds(1))
        .andThen(
            Commands.runOnce(
                () -> {
                  state.startAngle = gyro.getAngle().getRadians();
                  state.startPositions =
                      new double[] {
                        modules[0].getCurrentPosition().distanceMeters,
                        modules[1].getCurrentPosition().distanceMeters,
                        modules[2].getCurrentPosition().distanceMeters,
                        modules[3].getCurrentPosition().distanceMeters
                      };
                }))
        .andThen(Commands.waitSeconds(duration))
        .andThen(
            Commands.runOnce(
                () -> {
                  state.endAngle = gyro.getAngle().getRadians();
                  state.endPositions =
                      new double[] {
                        modules[0].getCurrentPosition().distanceMeters,
                        modules[1].getCurrentPosition().distanceMeters,
                        modules[2].getCurrentPosition().distanceMeters,
                        modules[3].getCurrentPosition().distanceMeters
                      };
                  double gyroDelta = Math.abs(state.endAngle - state.startAngle);
                  for (int i = 0; i < modules.length; i++) {
                    double wheelDelta = Math.abs(state.endPositions[i] - state.startPositions[i]);
                    wheelDelta /= SwerveConstants.BACK_LEFT_TRANSLATION.getNorm();
                    System.out.println(
                        "Module " + (i + 1) + " Coefficient: " + gyroDelta / wheelDelta);
                  }
                  System.out.println();
                }))
        .withName("Drive Conversion Factor Finder");
  }

  public Command backup() {
    return Commands.run(
            () -> {
              drive(new ChassisSpeeds(-2, 0, 0));
            },
            this)
        .withTimeout(.25)
        .withName("Backup");
  }

  public Command makePushable(Rotation2d pushDirection) {
    return Commands.run(
            () -> {
              FRONT_LEFT_MODULE.getAngleMotor().setTarget(pushDirection.getDegrees());
              FRONT_RIGHT_MODULE.getAngleMotor().setTarget(pushDirection.getDegrees());
              BACK_LEFT_MODULE.getAngleMotor().setTarget(pushDirection.getDegrees());
              BACK_RIGHT_MODULE.getAngleMotor().setTarget(pushDirection.getDegrees());
            },
            this)
        .beforeStarting(
            () -> {
              FRONT_LEFT_MODULE.setTargetState(new SwerveModuleState(0, pushDirection));
              FRONT_RIGHT_MODULE.setTargetState(new SwerveModuleState(0, pushDirection));
              BACK_LEFT_MODULE.setTargetState(new SwerveModuleState(0, pushDirection));
              BACK_RIGHT_MODULE.setTargetState(new SwerveModuleState(0, pushDirection));
            })
        .withName("Pushable: " + pushDirection.getDegrees() + " Degrees");
  }

  public Command xLock() {
    return Commands.run(
            () -> {
              FRONT_LEFT_MODULE.setTargetState(
                  new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
              FRONT_RIGHT_MODULE.setTargetState(
                  new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
              BACK_LEFT_MODULE.setTargetState(
                  new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
              BACK_RIGHT_MODULE.setTargetState(
                  new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            },
            this)
        .withName("Wheel Lock");
  }

  private ChassisSpeeds calculateVelRobotRel(XboxController xbox) {
    double speedCoefficient = Math.max(1 - xbox.getLeftTriggerAxis(), MIN_COEFFICIENT);
    Rotation2d currentHeading = estimator.getEstimatedPosition().getRotation();
    targetHeading =
        Rotation2d.fromRadians(
            targetHeading.getRadians()
                - withHardDeadzone(xbox.getRightX(), 0.1)
                    * speedCoefficient
                    * MAX_SPEEDS.omegaRadiansPerSecond
                    * 0.02);
    double rotational =
        headingFeedback.calculate(currentHeading.getRadians(), targetHeading.getRadians());
    if (headingFeedback.atGoal()) {
      rotational = 0;
    }
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      speedCoefficient *= -1;
    }
    double forward =
        speedCoefficient * withHardDeadzone(xbox.getLeftY(), 0.1) * MAX_SPEEDS.vxMetersPerSecond;
    double sideways =
        speedCoefficient * withHardDeadzone(xbox.getLeftX(), 0.1) * MAX_SPEEDS.vyMetersPerSecond;
    ChassisSpeeds fieldRel = new ChassisSpeeds(forward, sideways, rotational);
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRel, currentHeading);
  }

  private ChassisSpeeds applySkewCorrection(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    double angle = SKEW_COEFFICIENT * speeds.omegaRadiansPerSecond;
    return new ChassisSpeeds(
        Math.cos(angle) * speeds.vxMetersPerSecond - Math.sin(angle) * speeds.vyMetersPerSecond,
        Math.sin(angle) * speeds.vxMetersPerSecond + Math.cos(angle) * speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond);
  }

  private void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_MODULE_SPEED);
    speeds = kinematics.toChassisSpeeds(states);
    HoundLog.log("Swerve", "Target Speed", speeds);
    states = kinematics.toSwerveModuleStates(applySkewCorrection(speeds));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_MODULE_SPEED);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(states[i]);
    }
  }

  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  private void resetPose(Pose2d pose) {
    estimator.resetPosition(gyro.getAngle(), getModulePositions(), pose);
    targetHeading = pose.getRotation();
  }

  private ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getCurrentState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {

      states[i] = modules[i].getCurrentPosition();
    }
    return states;
  }

  @Override
  public void periodic() {
    estimator.update(gyro.getAngle(), getModulePositions());
    for (Limelight camera : tagCameras) {
      PoseEstimate estimate = camera.getPoseMT1();
      if (estimate.exists() && (estimate.averageDistance() < 2)) {
        estimator.addVisionMeasurement(
            estimate.pose(), Timer.getFPGATimestamp() - estimate.latencySeconds());
      }
    }
    for (SwerveModule module : modules) {
      module.periodic();
    }
  }

  @Override
  public void log(String path) {
    HoundLog.log(path, "Modules", getModuleStates());
    HoundLog.log(path, "Speeds", getSpeeds());
    HoundLog.log(path, "Pose", estimator.getEstimatedPosition());
    HoundLog.log(path, "Target Heading", targetHeading);
    HoundLog.log(path, "Gyro Angle", gyro.getAngle());
    HoundLog.log(path, "Front Left Module", modules[0]);
    HoundLog.log(path, "Front Right Module", modules[1]);
    HoundLog.log(path, "Back Left Module", modules[2]);
    HoundLog.log(path, "Back Right Module", modules[3]);
    HoundLog.log(path, "Gyro", gyro);
    HoundLog.log(path, "At Target Pose", poseFeedback.atTarget());
    HoundLog.log(path, "Target Pose", targetPose);
    for (Limelight camera : tagCameras) {
      HoundLog.log(path, camera.getName(), camera);
    }
  }

  public static enum Alignment {
    Right,
    Middle,
    Left,
    Top,
    Bottom;
  }
}
