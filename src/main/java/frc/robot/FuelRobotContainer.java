package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.VelocityMech;
import frc.robot.subsystems.VelocitySubsystem;
import frc.robot.subsystems.PositionMech;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopShootCommand;

/**
 * Robot with Fuel Shooter
 */
public class FuelRobotContainer extends RobotContainer {
  private final CANBus canBus = new CANBus(RobotConfig.FuelRobot.systemCANBus);
  private final VelocityMech intake;
  private final PositionMech tilter;
  private final VelocityMech indexer;
  private final VelocityMech feeder;
  private final VelocitySubsystem shooter;
  private final PositionMech hood;

  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

  public FuelRobotContainer(DrivetrainConfig config, AprilTagFieldLayout fieldLayout) {
    super(config, fieldLayout);

    int connectedJoystickCount = connectedJoystickCount();
    useTwoControllers = connectedJoystickCount == 2;

    intake = new VelocityMech(canBus, "Intake", IntakeConstants.MotorId);
    tilter = new PositionMech(canBus, "Tilter", IntakeConstants.TiltMotorId);
    indexer = new VelocityMech(canBus, "Indexer", IndexerConstants.IndexerId);
    feeder = new VelocityMech(canBus, "Feeder", IndexerConstants.FeederId);
    shooter = new VelocitySubsystem(canBus, "Shooter", ShooterConstants.RightMotorId, ShooterConstants.LeftMotorId);
    hood = new PositionMech(canBus, "Hood", ShooterConstants.HoodMotorId);

    Supplier<Pose2d> goalPoseSupplier = () -> new Pose2d(Units.feetToMeters(5), Units.feetToMeters(3), Rotation2d.fromDegrees(90));
    Supplier<Pose2d> poseProvider = drivetrain::getPose;

    configureBindings();

    storeParameters();
  }

  public void initRobot() {
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    CommandXboxController controller = useTwoControllers ? operatorController : driverController;

    // Intake control
    controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new IntakeCommand(intake, IntakeConstants.InSpeed));

    // Intake Tilt control
    controller.povLeft().whileTrue(new RunCommand(() -> tilter.jogDown(IntakeConstants.TiltStep), tilter));
    controller.povRight().whileTrue(new RunCommand(() -> tilter.jogUp(IntakeConstants.TiltStep), tilter));

    // Feeder control
    controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new FeederCommand(feeder, IndexerConstants.FeederSpeed, Constants.Forward));

    // Indexer control
    controller.leftBumper().whileTrue(new IndexerCommand(indexer, IndexerConstants.InSpeed, Constants.Backward));

    // Shooter control
    controller.a().onTrue(new ShootCommand(shooter, ShooterConstants.Speed));
    controller.b().onTrue(new StopShootCommand(shooter));

    // Shooter Hood
    controller.povUp().whileTrue(new RunCommand(() -> hood.jogUp(ShooterConstants.HoodStep), hood));
    controller.povDown().whileTrue(new RunCommand(() -> hood.jogDown(ShooterConstants.HoodStep), hood));

    // Fetch parameters
    controller.start().toggleOnTrue(new Command() {
        @Override public void initialize() {
          fetchParameters();
        }
        @Override public boolean isFinished() {
          return true;
        }
    });
    // Update parameters
    controller.back().toggleOnTrue(new Command() {
        @Override public void initialize() {
          storeParameters();
        }
        @Override public void execute() {
           System.out.println("execute");
        }
        @Override public boolean isFinished() {
          return true;
        }
    });
  }

  public void storeParameters()
  {
    System.out.println("storeParameters");

    intake.putParams();
    tilter.putParams();
    indexer.putParams();
    feeder.putParams();
    shooter.putParams();
    hood.putParams();

  }

  public void fetchParameters()
  {
    System.out.println("fetchParameters");

    intake.getParams();
    tilter.getParams();
    indexer.getParams();
    feeder.getParams();
    shooter.getParams();
    hood.getParams();
  }

  protected int connectedJoystickCount()
  {
    int connectedJoystickCount = 0;
    for (int i = 0; i < 6; ++i)
      if (DriverStation.isJoystickConnected(i))
        connectedJoystickCount++;
    return connectedJoystickCount;
  }

}
