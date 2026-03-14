package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivetrain.DrivetrainConfig;

/**
 * Robot with Coral Elevator — drivetrain-only mode.
 * No mechanism bindings (elevator, tray) since this is drivetrain-only.
 */
public class CoralRobotContainer extends RobotContainer {

  public CoralRobotContainer(DrivetrainConfig config, AprilTagFieldLayout fieldLayout) {
    super(config, fieldLayout);
  }

  @Override
  protected void configureBindings() {
    // Do NOT call super.configureBindings() — drivetrain-only bindings below.

    double maxSpeed = drivetrain.getConfig().maxSpeedMps;
    double maxAngularRate = drivetrain.getConfig().maxAngularRateRadPerSec;

    // Default command: left stick translate, right stick X rotate (field-centric)
    drivetrain.setDefaultCommand(
        drivetrain.driveFieldCentric(
            () -> {
              refreshPrefsCache();
              updateSlewRates();
              double speedScale = getSpeedScale();
              return translationXLimiter.calculate(getClampedStick()[0]) * maxSpeed * speedScale;
            },
            () -> {
              double speedScale = getSpeedScale();
              return translationYLimiter.calculate(getClampedStick()[1]) * maxSpeed * speedScale;
            },
            () -> {
              double rotScale = getRotationScale();
              return rotationLimiter.calculate(
                      InputProcessing.applyInputCurve(
                          -driverController.getRightX(), cachedDeadband, cachedRotationExpo))
                  * maxAngularRate
                  * rotScale;
            }));

    // Idle while disabled
    final var idle = new com.ctre.phoenix6.swerve.SwerveRequest.Idle();
    edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Left trigger = brake (X-lock wheels)
    driverController.leftTrigger(0.5).whileTrue(drivetrain.brake());

    // D-pad down = reset heading
    driverController.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.resetHeading()));
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
