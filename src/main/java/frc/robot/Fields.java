package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/** Field layout constants — competition field only. */
public final class Fields {
  public static final AprilTagFieldLayout COMPETITION =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private Fields() {}
}
