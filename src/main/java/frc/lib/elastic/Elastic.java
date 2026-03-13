package frc.lib.elastic;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

/**
 * Utility class for sending notifications and selecting tabs on the Elastic dashboard.
 * Elastic reads these values from NetworkTables.
 */
public final class Elastic {

  private static final StringTopic notificationTopic =
      NetworkTableInstance.getDefault().getStringTopic("/Elastic/RobotNotifications");
  private static final StringPublisher notificationPublisher = notificationTopic.publish();

  private static final StringTopic selectedTabTopic =
      NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
  private static final StringPublisher selectedTabPublisher = selectedTabTopic.publish();

  private Elastic() {}

  /** Sends a notification to the Elastic dashboard. */
  public static void sendNotification(ElasticNotification notification) {
    notificationPublisher.set(notification.toJson());
  }

  /** Selects a tab on the Elastic dashboard by name. */
  public static void selectTab(String tabName) {
    selectedTabPublisher.set(tabName);
  }
}
