package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.ModuleConfig;
import frc.lib.drivetrain.PIDGains;
import frc.lib.drivetrain.VisionConfig;

/**
 * Static DrivetrainConfig instances for each robot. CAN IDs and encoder offsets sourced from
 * RobotConfig.java. Shared tuning (gearing, PID, current limits) from new/ v0.5.
 */
public final class Robots {

  public static final DrivetrainConfig CORAL =
      DrivetrainConfig.builder()
          .canBus("CANdace")
          .pigeonId(24)
          .frontLeft(
              new ModuleConfig(
                  /* drive */ 7, /* steer */ 8, /* encoder */ 23,
                  /* offset */ 0.121337890625,
                  /* x */ 0.2921, /* y */ 0.2921,
                  /* invDrive */ false, /* invSteer */ false, /* invEnc */ false))
          .frontRight(
              new ModuleConfig(
                  /* drive */ 1, /* steer */ 2, /* encoder */ 20,
                  /* offset */ -0.294921875,
                  /* x */ 0.2921, /* y */ -0.2921,
                  /* invDrive */ true, /* invSteer */ false, /* invEnc */ false))
          .backLeft(
              new ModuleConfig(
                  /* drive */ 5, /* steer */ 6, /* encoder */ 22,
                  /* offset */ 0.040771484375,
                  /* x */ -0.2921, /* y */ 0.2921,
                  /* invDrive */ false, /* invSteer */ false, /* invEnc */ false))
          .backRight(
              new ModuleConfig(
                  /* drive */ 3, /* steer */ 4, /* encoder */ 21,
                  /* offset */ -0.376953125,
                  /* x */ -0.2921, /* y */ -0.2921,
                  /* invDrive */ true, /* invSteer */ false, /* invEnc */ false))
          .gearing(6.394736842105262, 12.1, 4.5, 0.0508)
          .speed(4.99, 0.75 * 2 * Math.PI)
          .steerPID(new PIDGains(15, 0, 0.9, 0.1, 1.5, 0))
          .drivePID(new PIDGains(0.1, 0, 0, 0, 0.124, 0))
          .simSteerPID(new PIDGains(50, 0, 0.2, 0, 1.0, 0))
          .simDrivePID(new PIDGains(0.05, 0, 0, 0, 0.1, 0))
          .currentLimits(30, 25, 20, 120)
          .deadband(0.05, 0.1)
          .mass(74.0, 6.0)
          .camera(
              "OV9281", new Transform3d(0.3, 0, 0.25, new Rotation3d(0, Math.toRadians(-15), 0)))
          .visionConfig(VisionConfig.builder().enabledByDefault(false).build())
          .build();

  public static final DrivetrainConfig FUEL =
      DrivetrainConfig.builder()
          .canBus("Drivetrain")
          .pigeonId(20)
          .frontLeft(
              new ModuleConfig(
                  /* drive */ 2, /* steer */ 1, /* encoder */ 21,
                  /* offset */ 0.105224609375,
                  /* x */ 0.282575, /* y */ 0.282575,
                  /* invDrive */ false, /* invSteer */ false, /* invEnc */ false))
          .frontRight(
              new ModuleConfig(
                  /* drive */ 4, /* steer */ 3, /* encoder */ 22,
                  /* offset */ -0.12060546875,
                  /* x */ 0.282575, /* y */ -0.282575,
                  /* invDrive */ true, /* invSteer */ false, /* invEnc */ false))
          .backLeft(
              new ModuleConfig(
                  /* drive */ 6, /* steer */ 5, /* encoder */ 23,
                  /* offset */ -0.466796875,
                  /* x */ -0.282575, /* y */ 0.282575,
                  /* invDrive */ false, /* invSteer */ false, /* invEnc */ false))
          .backRight(
              new ModuleConfig(
                  /* drive */ 8, /* steer */ 7, /* encoder */ 24,
                  /* offset */ -0.037109375,
                  /* x */ -0.282575, /* y */ -0.282575,
                  /* invDrive */ true, /* invSteer */ false, /* invEnc */ false))
          .gearing(6.394736842105262, 12.1, 4.5, 0.0508)
          .speed(4.99, 0.75 * 2 * Math.PI)
          .steerPID(new PIDGains(100, 0, 0.5, 0.1, 1.5, 0))
          .drivePID(new PIDGains(0.1, 0, 0, 0, 0.124, 0))
          .simSteerPID(new PIDGains(50, 0, 0.2, 0, 1.0, 0))
          .simDrivePID(new PIDGains(0.05, 0, 0, 0, 0.1, 0))
          .currentLimits(30, 25, 20, 120)
          .deadband(0.05, 0.1)
          .mass(74.0, 6.0)
          .build();

  private Robots() {}
}
