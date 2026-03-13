package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivetrain.DriveState;
import frc.lib.drivetrain.DriveTelemetry;
import frc.lib.elastic.Elastic;
import frc.lib.elastic.ElasticNotification;
import frc.lib.elastic.ElasticNotification.NotificationLevel;

/** Publishes driver-relevant drivetrain data to SmartDashboard and Elastic notifications. */
public class DriverDashboard implements DriveTelemetry {

  private boolean lastBrownout = false;
  private boolean lastAllHealthy = true;

  @Override
  public void update(DriveState state) {
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
