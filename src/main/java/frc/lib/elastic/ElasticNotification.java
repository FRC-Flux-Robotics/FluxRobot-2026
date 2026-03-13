package frc.lib.elastic;

/**
 * A notification to be sent to the Elastic dashboard.
 * Based on the Elastic Dashboard robot-side API.
 */
public class ElasticNotification {

  private NotificationLevel level;
  private String title;
  private String description;
  private int displayTimeMillis;
  private double width = 350;
  private double height = -1;

  public enum NotificationLevel {
    INFO,
    WARNING,
    ERROR
  }

  public ElasticNotification() {
    this(NotificationLevel.INFO, "", "", 3000);
  }

  public ElasticNotification(NotificationLevel level, String title, String description) {
    this(level, title, description, 3000);
  }

  public ElasticNotification(
      NotificationLevel level, String title, String description, int displayTimeMillis) {
    this.level = level;
    this.title = title;
    this.description = description;
    this.displayTimeMillis = displayTimeMillis;
  }

  public ElasticNotification withLevel(NotificationLevel level) {
    this.level = level;
    return this;
  }

  public ElasticNotification withTitle(String title) {
    this.title = title;
    return this;
  }

  public ElasticNotification withDescription(String description) {
    this.description = description;
    return this;
  }

  public ElasticNotification withDisplaySeconds(double seconds) {
    this.displayTimeMillis = (int) (seconds * 1000);
    return this;
  }

  public NotificationLevel getLevel() {
    return level;
  }

  public String getTitle() {
    return title;
  }

  public String getDescription() {
    return description;
  }

  /** Serializes this notification to JSON for NetworkTables transport. */
  public String toJson() {
    return String.format(
        "{\"level\":\"%s\",\"title\":\"%s\",\"description\":\"%s\","
            + "\"displayTime\":%d,\"width\":%.1f,\"height\":%.1f}",
        level.name(),
        escapeJson(title),
        escapeJson(description),
        displayTimeMillis,
        width,
        height);
  }

  private static String escapeJson(String s) {
    if (s == null) return "";
    return s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n");
  }
}
