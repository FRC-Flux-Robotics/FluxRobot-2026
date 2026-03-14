package frc.lib.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.IntFunction;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class DrivetrainIOTalonFX implements DrivetrainIO {
  private final Supplier<SwerveDriveState> stateSupplier;
  private final double driveGearRatio;
  private final PhotonCamera[] cameras;
  private final PhotonPoseEstimator[] poseEstimators;
  private final PhotonPipelineResult[] latestCameraResults;

  // Cached StatusSignal references (per module × 5 signals + 1 gyro = 21 total)
  private final StatusSignal<?>[] drivePositionSignals = new StatusSignal<?>[4];
  private final StatusSignal<?>[] driveTempSignals = new StatusSignal<?>[4];
  private final StatusSignal<?>[] steerTempSignals = new StatusSignal<?>[4];
  private final StatusSignal<?>[] driveCurrentSignals = new StatusSignal<?>[4];
  private final StatusSignal<?>[] steerCurrentSignals = new StatusSignal<?>[4];
  private final StatusSignal<?> gyroYawSignal;
  private final BaseStatusSignal[] allSignals;

  // Round-robin vision processing
  private int visionCycleCounter = 0;

  public DrivetrainIOTalonFX(
      Supplier<SwerveDriveState> stateSupplier,
      IntFunction<SwerveModule<TalonFX, TalonFX, CANcoder>> moduleSupplier,
      double driveGearRatio,
      List<CameraConfig> cameraConfigs,
      AprilTagFieldLayout fieldLayout,
      StatusSignal<?> gyroYawSignal) {
    this.stateSupplier = stateSupplier;
    this.driveGearRatio = driveGearRatio;
    this.gyroYawSignal = gyroYawSignal;

    // Cache all StatusSignal references for batch refresh
    for (int i = 0; i < 4; i++) {
      SwerveModule<TalonFX, TalonFX, CANcoder> module = moduleSupplier.apply(i);
      TalonFX driveMotor = module.getDriveMotor();
      TalonFX steerMotor = module.getSteerMotor();

      drivePositionSignals[i] = driveMotor.getPosition();
      driveTempSignals[i] = driveMotor.getDeviceTemp();
      steerTempSignals[i] = steerMotor.getDeviceTemp();
      driveCurrentSignals[i] = driveMotor.getStatorCurrent();
      steerCurrentSignals[i] = steerMotor.getStatorCurrent();

      // Reduce CAN frame rates for signals we're not actively reading
      driveMotor.optimizeBusUtilization();
      steerMotor.optimizeBusUtilization();
      module.getEncoder().optimizeBusUtilization();
    }
    allSignals = new BaseStatusSignal[21];
    for (int i = 0; i < 4; i++) {
      allSignals[i * 5 + 0] = drivePositionSignals[i];
      allSignals[i * 5 + 1] = driveTempSignals[i];
      allSignals[i * 5 + 2] = steerTempSignals[i];
      allSignals[i * 5 + 3] = driveCurrentSignals[i];
      allSignals[i * 5 + 4] = steerCurrentSignals[i];
    }
    allSignals[20] = gyroYawSignal;

    if (cameraConfigs != null && fieldLayout != null) {
      cameras = new PhotonCamera[cameraConfigs.size()];
      poseEstimators = new PhotonPoseEstimator[cameraConfigs.size()];
      latestCameraResults = new PhotonPipelineResult[cameraConfigs.size()];
      for (int i = 0; i < cameraConfigs.size(); i++) {
        CameraConfig cfg = cameraConfigs.get(i);
        cameras[i] = new PhotonCamera(cfg.name());
        poseEstimators[i] =
            new PhotonPoseEstimator(
                fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cfg.robotToCamera());
        poseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      }
    } else {
      cameras = new PhotonCamera[0];
      poseEstimators = new PhotonPoseEstimator[0];
      latestCameraResults = new PhotonPipelineResult[0];
    }
  }

  @Override
  public PhotonPipelineResult getLatestResult(int cameraIndex) {
    if (cameraIndex >= 0 && cameraIndex < latestCameraResults.length) {
      return latestCameraResults[cameraIndex];
    }
    return null;
  }

  @Override
  public void updateInputs(DrivetrainIOInputsAutoLogged inputs) {
    // Batch-refresh all 21 CAN signals in one frame
    BaseStatusSignal.refreshAll(allSignals);

    SwerveDriveState state = stateSupplier.get();

    // Gyro
    inputs.gyroYawDeg = state.Pose.getRotation().getDegrees();
    inputs.gyroRateDegPerSec = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);
    inputs.gyroHealthy = gyroYawSignal.getStatus().isOK();

    // Odometry
    inputs.odometryPeriodSec = state.OdometryPeriod;

    // Per-module data (read from cached signals — just field reads after refreshAll)
    for (int i = 0; i < 4; i++) {
      inputs.steerAngleDeg[i] = state.ModuleStates[i].angle.getDegrees();
      inputs.driveVelocityMps[i] = state.ModuleStates[i].speedMetersPerSecond;
      inputs.drivePositionRad[i] =
          drivePositionSignals[i].getValueAsDouble() * 2.0 * Math.PI / driveGearRatio;

      inputs.driveTempC[i] = driveTempSignals[i].getValueAsDouble();
      inputs.steerTempC[i] = steerTempSignals[i].getValueAsDouble();
      inputs.driveCurrentA[i] = driveCurrentSignals[i].getValueAsDouble();
      inputs.steerCurrentA[i] = steerCurrentSignals[i].getValueAsDouble();

      // Health: check signal status for fault detection
      inputs.driveHealthy[i] =
          driveCurrentSignals[i].getStatus().isOK()
              && drivePositionSignals[i].getStatus().isOK();
      inputs.steerHealthy[i] = steerCurrentSignals[i].getStatus().isOK();
    }

    // Battery voltage
    inputs.batteryVoltage = RobotController.getBatteryVoltage();

    // Vision
    int camCount = cameras.length;
    if (camCount > 0) {
      // Allocate arrays only once, reuse and zero each cycle
      if (inputs.visionConnected.length != camCount) {
        inputs.visionConnected = new boolean[camCount];
        inputs.visionHasEstimate = new boolean[camCount];
        inputs.visionPoseX = new double[camCount];
        inputs.visionPoseY = new double[camCount];
        inputs.visionPoseRotDeg = new double[camCount];
        inputs.visionTimestampSec = new double[camCount];
        inputs.visionTagCount = new int[camCount];
        inputs.visionAmbiguity = new double[camCount];
        inputs.visionAvgTagDistM = new double[camCount];
        inputs.visionPoseZ = new double[camCount];
      } else {
        Arrays.fill(inputs.visionConnected, false);
        Arrays.fill(inputs.visionHasEstimate, false);
        Arrays.fill(inputs.visionPoseX, 0);
        Arrays.fill(inputs.visionPoseY, 0);
        Arrays.fill(inputs.visionPoseRotDeg, 0);
        Arrays.fill(inputs.visionTimestampSec, 0);
        Arrays.fill(inputs.visionTagCount, 0);
        Arrays.fill(inputs.visionAmbiguity, 0);
        Arrays.fill(inputs.visionAvgTagDistM, 0);
        Arrays.fill(inputs.visionPoseZ, 0);
      }

      // Round-robin: process one camera per cycle, check connectivity for all
      int activeCamera = visionCycleCounter % camCount;
      visionCycleCounter++;

      for (int i = 0; i < camCount; i++) {
        inputs.visionConnected[i] = cameras[i].isConnected();

        // Only fully process the active camera this cycle
        if (i != activeCamera) {
          continue;
        }

        List<PhotonPipelineResult> results = cameras[i].getAllUnreadResults();
        if (results.isEmpty()) {
          continue;
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);
        latestCameraResults[i] = latest;
        Optional<EstimatedRobotPose> estimate = poseEstimators[i].update(latest);
        if (estimate.isPresent()) {
          EstimatedRobotPose est = estimate.get();
          inputs.visionHasEstimate[i] = true;
          inputs.visionPoseX[i] = est.estimatedPose.getX();
          inputs.visionPoseY[i] = est.estimatedPose.getY();
          inputs.visionPoseRotDeg[i] = est.estimatedPose.getRotation().toRotation2d().getDegrees();
          inputs.visionPoseZ[i] = est.estimatedPose.getZ();
          inputs.visionTimestampSec[i] = est.timestampSeconds;
          inputs.visionTagCount[i] = est.targetsUsed.size();
          if (!est.targetsUsed.isEmpty()) {
            // Worst-case ambiguity across all targets (not just first)
            double maxAmbiguity = 0;
            double totalDist = 0;
            for (var target : est.targetsUsed) {
              maxAmbiguity = Math.max(maxAmbiguity, target.getPoseAmbiguity());
              totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
            }
            inputs.visionAmbiguity[i] = maxAmbiguity;
            inputs.visionAvgTagDistM[i] = totalDist / est.targetsUsed.size();
          }
        }
      }
    }
  }
}
