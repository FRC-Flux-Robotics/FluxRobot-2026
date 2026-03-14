package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivetrain.DriveState;
import frc.lib.drivetrain.DriveTelemetry;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.elastic.Elastic;
import frc.lib.elastic.ElasticNotification;
import frc.lib.elastic.ElasticNotification.NotificationLevel;

/** Publishes driver-relevant drivetrain data to SmartDashboard and Elastic notifications. */
public class DriverDashboard implements DriveTelemetry {

  private final Field2d field = new Field2d();
  private boolean lastBrownout = false;
  private boolean lastAllHealthy = true;

  public DriverDashboard(DrivetrainConfig config) {
    SmartDashboard.putData("Field", field);

    // Publish config parameters once
    SmartDashboard.putNumber("Config/Max Speed (m_s)", config.maxSpeedMps);
    SmartDashboard.putNumber("Config/Max Angular Rate (rad_s)", config.maxAngularRateRadPerSec);
    SmartDashboard.putNumber("Config/Drive Gear Ratio", config.driveGearRatio);
    SmartDashboard.putNumber("Config/Wheel Radius (m)", config.wheelRadiusMeters);
    SmartDashboard.putNumber("Config/Drive Stator Limit (A)", config.driveStatorCurrentLimit);
    SmartDashboard.putNumber("Config/Drive Supply Limit (A)", config.driveSupplyCurrentLimit);
    SmartDashboard.putNumber("Config/Steer Stator Limit (A)", config.steerStatorCurrentLimit);
    SmartDashboard.putNumber("Config/Slip Current (A)", config.slipCurrentAmps);
    SmartDashboard.putString("Config/CAN Bus", config.canBus);
    SmartDashboard.putNumber("Config/Mass (kg)", config.massKg);
  }

  @Override
  public void update(DriveState state) {
    // Update field map with robot pose
    field.setRobotPose(state.pose());

    // Numeric telemetry — Elastic reads SmartDashboard natively
    SmartDashboard.putNumber("Speed %", state.speedPercent());
    SmartDashboard.putNumber("Total Current A", state.totalCurrentA());
    SmartDashboard.putBoolean("Low Battery", state.brownoutActive());
    SmartDashboard.putBoolean("All Healthy", state.allHealthy());
    SmartDashboard.putString("Status", state.statusMessage());

    // Elastic notifications for driver-critical state changes
    if (state.brownoutActive() && !lastBrownout) {
      Elastic.sendNotification(
          new ElasticNotification(
              NotificationLevel.ERROR,
              "Low Battery",
              String.format("Battery voltage low (%.1fV) — brownout active!", state.batteryVoltage()),
              5000));
    }

    if (!state.allHealthy() && lastAllHealthy) {
      Elastic.sendNotification(
          new ElasticNotification(
              NotificationLevel.WARNING,
              "Module Unhealthy",
              state.statusMessage(),
              4000));
    }

    lastBrownout = state.brownoutActive();
    lastAllHealthy = state.allHealthy();
  }
}
