#include "pumps/vacuflush_pump_monitor.h"

#include <ArduinoJson.h>

#include "Debug.h"
#include "sensesp/system/lambda_consumer.h"

namespace app {

namespace {

constexpr bool kDefaultManualEnabled = true;
constexpr float kDefaultMaxRunTimeSeconds = 120.0f;
constexpr float kDefaultMinStopTimeSeconds = 5.0f;
constexpr float kDefaultMinIdleTimeSeconds = 300.0f;
constexpr float kDefaultMinFlushTimeSeconds = 35.0f;
constexpr int kDefaultMaxConsecutiveShortCycles = 5;
constexpr int kDefaultMaxCyclesInWindow = 10;
constexpr int kDefaultMaxFlushesInWindow = 4;
constexpr const char* kDefaultShortCycleAlertSeverity = "alert";

constexpr unsigned long kMonitorEvaluationIntervalMs = 200;
constexpr unsigned long kRawRunInputReadIntervalMs = 100;
constexpr unsigned long kDisabledAlertDelayMs = 3000;
constexpr unsigned long kDisabledReportIntervalMs = 1000;
constexpr unsigned long kCycleWindowDurationMs = 60UL * 60UL * 1000UL;
constexpr unsigned long kCycleWindowHousekeepingIntervalMs = 60UL * 1000UL;
constexpr unsigned long kMaxIdleTimeForAverageMs = 24UL * 60UL * 60UL * 1000UL;
constexpr float kMinReportIntervalSeconds = 0.1f;
constexpr float kDefaultAggregateReportIntervalSeconds = 10.0f;
constexpr float kMinAggregateReportIntervalSeconds = 0.1f;
constexpr time_t kValidWallClockEpoch = 1704067200;  // 2024-01-01T00:00:00Z

float clamp_min_float(float value, float minimum) {
  return value < minimum ? minimum : value;
}

int clamp_min_int(int value, int minimum) {
  return value < minimum ? minimum : value;
}

}  // namespace

VacuflushPumpMonitor::VacuflushPumpMonitor(const String& pump_role,
                                           const String& pump_name,
                                           const String& config_path,
                                           int sense_pin, int relay_pin)
    : FileSystemSaveable(config_path),
      pump_role_(pump_role),
      pump_name_(pump_name),
      sense_pin_(sense_pin),
      relay_pin_(relay_pin),
      manual_enabled_(kDefaultManualEnabled),
      max_run_time_s_(kDefaultMaxRunTimeSeconds),
      min_stop_time_s_(kDefaultMinStopTimeSeconds),
      min_idle_time_s_(kDefaultMinIdleTimeSeconds),
      min_flush_time_s_(kDefaultMinFlushTimeSeconds),
      max_consecutive_short_cycles_(kDefaultMaxConsecutiveShortCycles),
      max_cycles_in_window_(kDefaultMaxCyclesInWindow),
      max_flushes_in_window_(kDefaultMaxFlushesInWindow),
      short_cycle_alert_severity_(kDefaultShortCycleAlertSeverity) {
  initialize_paths();
}

void VacuflushPumpMonitor::begin(float state_report_interval_seconds,
                                 float aggregate_report_interval_seconds) {
  if (started_) {
    return;
  }

  raw_run_input_ = new sensesp::DigitalInputState(sense_pin_, INPUT,
                                                  kRawRunInputReadIntervalMs);
  raw_run_input_->emit(digitalRead(sense_pin_) == HIGH);

  relay_output_ = new sensesp::DigitalOutput(relay_pin_);
  relay_drive_transform_ = new sensesp::LambdaTransform<bool, bool>(
      [](bool enabled_state) { return !enabled_state; });

  running_state_ = new sensesp::ObservableValue<bool>(false);
  enabled_state_ = new sensesp::ObservableValue<bool>(false);
  last_run_time_ = new sensesp::ObservableValue<float>(0.0f);
  last_idle_time_ = new sensesp::ObservableValue<float>(0.0f);
  disabled_time_ = new sensesp::ObservableValue<float>(0.0f);
  daily_cycle_count_ = new sensesp::ObservableValue<int>(0);
  short_cycle_count_value_ = new sensesp::ObservableValue<int>(0);
  average_run_time_ = new sensesp::ObservableValue<float>(0.0f);
  average_idle_time_ = new sensesp::ObservableValue<float>(0.0f);

  daily_stats_day_key_ = new sensesp::PersistingObservableValue<int>(
      -1, get_config_path() + "/stats/daily_stats_day_key");
  daily_flush_count_ = new sensesp::PersistingObservableValue<int>(
      0, get_config_path() + "/stats/daily_flush_count");
  total_flush_count_ = new sensesp::PersistingObservableValue<int>(
      0, get_config_path() + "/stats/total_flush_count");
      
  runtime_notification_ =
      new sensesp::ObservableValue<String>(make_cleared_notification_payload());
  cycling_notification_ =
      new sensesp::ObservableValue<String>(make_cleared_notification_payload());
  relay_notification_ =
      new sensesp::ObservableValue<String>(make_cleared_notification_payload());
  cycles_notification_ =
      new sensesp::ObservableValue<String>(make_cleared_notification_payload());
  flushes_notification_ =
      new sensesp::ObservableValue<String>(make_cleared_notification_payload());

  load();

  const unsigned long initial_state_interval_ms = static_cast<unsigned long>(
      clamp_min_float(state_report_interval_seconds, kMinReportIntervalSeconds) *
      1000.0f);
  const unsigned long initial_aggregate_interval_ms =
      static_cast<unsigned long>(
          clamp_min_float(aggregate_report_interval_seconds,
                          kMinAggregateReportIntervalSeconds) *
          1000.0f);

  auto* running_output =
      new sensesp::SKOutputBool(running_sk_path_, make_running_metadata());
  running_state_
      ->connect_to(new RepeatMutable<bool>(initial_state_interval_ms))
      ->connect_to(running_output);

  enabled_state_
      ->connect_to(new RepeatMutable<bool>(initial_state_interval_ms))
      ->connect_to(new sensesp::SKOutputBool(enabled_state_sk_path_, "",
                                             make_enabled_state_metadata()));

  enabled_state_->connect_to(relay_drive_transform_)->connect_to(relay_output_);

  last_run_time_->connect_to(new sensesp::SKOutputFloat(
      last_run_time_sk_path_, "", make_last_run_time_metadata()));

  last_idle_time_->connect_to(new sensesp::SKOutputFloat(
      last_idle_time_sk_path_, "", make_last_idle_time_metadata()));

  disabled_time_->connect_to(new sensesp::SKOutputFloat(
      disabled_time_sk_path_, "", make_disabled_time_metadata()));

  daily_cycle_count_
      ->connect_to(new RepeatMutable<int>(initial_aggregate_interval_ms))
      ->connect_to(new sensesp::SKOutputInt(
          daily_cycle_count_sk_path_, "", make_daily_cycle_count_metadata()));

  short_cycle_count_value_
      ->connect_to(new RepeatMutable<int>(initial_aggregate_interval_ms))
      ->connect_to(new sensesp::SKOutputInt(
          short_cycle_count_sk_path_, "", make_short_cycle_count_metadata()));

  average_run_time_
      ->connect_to(new RepeatMutable<float>(initial_aggregate_interval_ms))
      ->connect_to(new sensesp::SKOutputFloat(
          average_run_time_sk_path_, "", make_average_run_time_metadata()));

  average_idle_time_
      ->connect_to(new RepeatMutable<float>(initial_aggregate_interval_ms))
      ->connect_to(new sensesp::SKOutputFloat(
          average_idle_time_sk_path_, "", make_average_idle_time_metadata()));

  daily_flush_count_
      ->connect_to(new RepeatMutable<int>(initial_aggregate_interval_ms))
      ->connect_to(new sensesp::SKOutputInt(
          daily_flush_count_sk_path_, "", make_daily_flush_count_metadata()));

  total_flush_count_
      ->connect_to(new RepeatMutable<int>(initial_aggregate_interval_ms))
      ->connect_to(new sensesp::SKOutputInt(
          total_flush_count_sk_path_, "", make_total_flush_count_metadata()));

  runtime_notification_->connect_to(new sensesp::SKOutputRawJson(
      runtime_notification_sk_path_, "", make_runtime_notification_metadata()));

  cycling_notification_->connect_to(new sensesp::SKOutputRawJson(
      cycling_notification_sk_path_, "", make_cycling_notification_metadata()));

  relay_notification_->connect_to(new sensesp::SKOutputRawJson(
      relay_notification_sk_path_, "", make_relay_notification_metadata()));

  cycles_notification_->connect_to(new sensesp::SKOutputRawJson(
      cycles_notification_sk_path_, "", make_cycles_notification_metadata()));

  flushes_notification_->connect_to(new sensesp::SKOutputRawJson(
      flushes_notification_sk_path_, "", make_flushes_notification_metadata()));

  auto* put_listener =
      new MutableBoolSKPutRequestListener(enabled_state_sk_path_);

  put_listener->connect_to(new sensesp::LambdaConsumer<bool>(
      [this](bool enabled) {
        LOG_W("%s: received PUT enabled=%s", pump_name_.c_str(),
              enabled ? "true" : "false");
        this->set_manual_enabled(enabled);
      }));

  const unsigned long now = millis();
  logical_running_ = current_raw_running();
  cycle_start_ms_ = logical_running_ ? now : 0;
  idle_timer_active_ = !logical_running_;
  idle_start_ms_ = logical_running_ ? 0 : now;
  pending_cycle_idle_duration_ms_ = 0;
  cycle_window_epoch_ms_ = now;
  last_cycle_window_housekeeping_ms_ = now;
  stop_candidate_ms_ = 0;
  disabled_since_ms_ = effective_enabled() ? 0 : now;
  last_disabled_report_ms_ = 0;
  update_daily_cycle_boundary(time(nullptr));

  started_ = true;

  *running_state_ = logical_running_;
  *enabled_state_ = effective_enabled();
  *disabled_time_ = effective_enabled()
                        ? 0.0f
                        : static_cast<float>(now - disabled_since_ms_) / 1000.0f;

  evaluation_event_ = sensesp::event_loop()->onRepeat(
      kMonitorEvaluationIntervalMs, [this]() { this->evaluate(); });

  LOG_I("%s: monitor started on sense pin %d relay pin %d", pump_name_.c_str(),
        sense_pin_, relay_pin_);
}

bool VacuflushPumpMonitor::current_raw_running() const {
  return raw_run_input_ != nullptr && raw_run_input_->get();
}

void VacuflushPumpMonitor::initialize_paths() {
  running_sk_path_ = make_pump_path("running");
  enabled_state_sk_path_ = make_switch_path("enabled");
  last_run_time_sk_path_ = make_pump_path("lastCycleRunTime");
  last_idle_time_sk_path_ = make_pump_path("lastCycleIdleTime");
  disabled_time_sk_path_ = make_pump_path("disabledTime");
  daily_cycle_count_sk_path_ = make_pump_path("dailyCycleCount");
  short_cycle_count_sk_path_ = make_pump_path("shortCycleCount");
  average_run_time_sk_path_ = make_pump_path("averageCycleRunTime");
  average_idle_time_sk_path_ = make_pump_path("averageCycleIdleTime");
  daily_flush_count_sk_path_ = make_pump_path("dailyFlushCount");
  total_flush_count_sk_path_ = make_pump_path("totalFlushCount");
  runtime_notification_sk_path_ = make_notification_path("runtime");
  cycling_notification_sk_path_ = make_notification_path("cycling");
  relay_notification_sk_path_ = make_notification_path("relay");
  cycles_notification_sk_path_ = make_notification_path("cycles");
  flushes_notification_sk_path_ = make_notification_path("flushes");
}

String VacuflushPumpMonitor::make_pump_path(const String& suffix) const {
  return "electrical.pumps.sanitation.vacuum." + pump_role_ + "." + suffix;
}

String VacuflushPumpMonitor::make_switch_path(const String& suffix) const {
  return "electrical.switches.sanitation.vacuum." + pump_role_ + "." + suffix;
}

String VacuflushPumpMonitor::make_notification_path(const String& suffix) const {
  return "notifications.sanitation.vacuum." + pump_role_ + "." + suffix;
}

int VacuflushPumpMonitor::day_key_from_time(time_t now_seconds) const {
  if (now_seconds < kValidWallClockEpoch) {
    return -1;
  }

  struct tm local_time {};
  if (localtime_r(&now_seconds, &local_time) == nullptr) {
    return -1;
  }

  return (local_time.tm_year + 1900) * 1000 + local_time.tm_yday;
}

void VacuflushPumpMonitor::update_daily_cycle_boundary(time_t now_seconds) {
  const int day_key = day_key_from_time(now_seconds);
  if (day_key < 0) {
    return;
  }

  if (daily_stats_day_key_ == nullptr) {
    return;
  }

  if (daily_stats_day_key_->get() < 0) {
    *daily_stats_day_key_ = day_key;
    return;
  }

  if (day_key != daily_stats_day_key_->get()) {
    *daily_stats_day_key_ = day_key;
    *daily_cycle_count_ = 0;
    *daily_flush_count_ = 0;
    LOG_I("%s: reset daily cycle and flush counts at midnight boundary",
          pump_name_.c_str());
  }
}

String VacuflushPumpMonitor::make_notification_payload(const char* state,
                                                       const String& message,
                                                       bool audible) const {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root["state"] = state;
  JsonArray method = root["method"].to<JsonArray>();
  method.add("visual");
  if (audible) {
    method.add("sound");
  }
  root["message"] = message;

  String payload;
  serializeJson(root, payload);
  return payload;
}

String VacuflushPumpMonitor::make_cleared_notification_payload() const {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root["state"] = "normal";
  root["message"] = "";

  String payload;
  serializeJson(root, payload);
  return payload;
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_running_metadata() const {
  return new sensesp::SKMetadata(
      "", pump_name_ + " Running",
      "True when the vacuum pump motor input indicates the pump is running.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_enabled_state_metadata() const {
  return new sensesp::SKMetadata(
      "", pump_name_ + " Enabled State",
      "Boolean state of the vacuum pump enable switch where true means enabled and false means disabled.",
      "", -1.0f, true);
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_last_run_time_metadata() const {
  return new sensesp::SKMetadata(
      "s", pump_name_ + " Last Cycle Run Time",
      "Duration in seconds of the most recently completed pump run cycle.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_last_idle_time_metadata() const {
  return new sensesp::SKMetadata(
      "s", pump_name_ + " Last Cycle Idle Time",
      "Idle time in seconds measured before the current or most recent pump run cycle started.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_disabled_time_metadata() const {
  return new sensesp::SKMetadata(
      "s", pump_name_ + " Disabled Time",
      "Elapsed time in seconds that the pump has continuously remained disabled.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_daily_cycle_count_metadata()
    const {
  return new sensesp::SKMetadata(
      "count", pump_name_ + " Daily Cycle Count",
      "Number of completed pump cycles since the most recent midnight boundary.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_short_cycle_count_metadata()
    const {
  return new sensesp::SKMetadata(
      "count", pump_name_ + " Short Cycle Count",
      "Current count of consecutive short pump cycles.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_average_run_time_metadata()
    const {
  return new sensesp::SKMetadata(
      "s", pump_name_ + " Average Cycle Run Time",
      "Average completed pump run duration in seconds since device boot.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_average_idle_time_metadata()
    const {
  return new sensesp::SKMetadata(
      "s", pump_name_ + " Average Cycle Idle Time",
      "Average pump idle duration in seconds since device boot.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_daily_flush_count_metadata()
    const {
  return new sensesp::SKMetadata(
      "count", pump_name_ + " Daily Flush Count",
      "Number of detected flush cycles since the most recent midnight boundary.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_total_flush_count_metadata()
    const {
  return new sensesp::SKMetadata(
      "count", pump_name_ + " Total Flush Count",
      "Total number of detected flush cycles since the counter was initialized.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_runtime_notification_metadata()
    const {
  return new sensesp::SKMetadata(
      "", pump_name_ + " Runtime Alert",
      "Notification raised when the pump exceeds its configured maximum run time.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_cycling_notification_metadata()
    const {
  return new sensesp::SKMetadata(
      "", pump_name_ + " Short Cycle Alert",
      "Notification raised when the pump exceeds its configured consecutive short cycle threshold.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_relay_notification_metadata()
    const {
  return new sensesp::SKMetadata(
      "", pump_name_ + " Relay Fault Alert",
      "Notification raised when the pump still reports running after the relay has been commanded to disable it.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_cycles_notification_metadata()
    const {
  return new sensesp::SKMetadata(
      "", pump_name_ + " Cycle Rate Alert",
      "Notification raised when the pump exceeds the configured maximum completed cycles within the last 60 minutes.");
}

sensesp::SKMetadata* VacuflushPumpMonitor::make_flushes_notification_metadata()
    const {
  return new sensesp::SKMetadata(
      "", pump_name_ + " Flush Rate Alert",
      "Notification raised when the pump exceeds the configured maximum detected flushes within the last 60 minutes.");
}

void VacuflushPumpMonitor::republish_all_outputs() {
  onSKServerConnect();
}

void VacuflushPumpMonitor::onSKServerConnect() {
  if (!started_) {
    return;
  }

  running_state_->notify();
  enabled_state_->notify();
  last_run_time_->notify();
  last_idle_time_->notify();
  disabled_time_->notify();
  daily_cycle_count_->notify();
  short_cycle_count_value_->notify();
  average_run_time_->notify();
  average_idle_time_->notify();
  daily_flush_count_->notify();
  total_flush_count_->notify();

  runtime_notification_->notify();
  cycling_notification_->notify();
  relay_notification_->notify();
  cycles_notification_->notify();
  flushes_notification_->notify();
}

void VacuflushPumpMonitor::set_notification(
    sensesp::ObservableValue<String>* producer, String& active_message,
    const char* state, const String& message, bool audible) {
  if (message == active_message) {
    return;
  }

  producer->set(make_notification_payload(state, message, audible));
  active_message = message;
  LOG_W("%s: %s", pump_name_.c_str(), message.c_str());
}

void VacuflushPumpMonitor::clear_notification(
    sensesp::ObservableValue<String>* producer, String& active_message) {
  if (!active_message.isEmpty()) {
    LOG_I("%s: cleared notification: %s", pump_name_.c_str(),
          active_message.c_str());
  }
  producer->set(make_cleared_notification_payload());
  active_message = "";
}

void VacuflushPumpMonitor::clear_all_notifications() {
  clear_notification(runtime_notification_, active_runtime_alert_message_);
  clear_notification(cycling_notification_, active_cycling_alert_message_);
  clear_notification(relay_notification_, active_relay_alert_message_);
  clear_notification(cycles_notification_, active_cycles_alert_message_);
  clear_notification(flushes_notification_, active_flushes_alert_message_);
}

void VacuflushPumpMonitor::set_manual_enabled(bool enabled, bool persist) {
  manual_enabled_ = enabled;

  if (enabled) {
    fault_latched_ = false;
    clear_all_notifications();
    reset_cycle_window(millis());
  }

  if (effective_enabled()) {
    disabled_since_ms_ = 0;
    *disabled_time_ = 0.0f;
  } else {
    disabled_since_ms_ = millis();
    last_disabled_report_ms_ = 0;
  }

  *enabled_state_ = effective_enabled();

  if (persist) {
    save();
  }

  LOG_W("%s: enabled set to %s", pump_name_.c_str(),
        effective_enabled() ? "true" : "false");
}

void VacuflushPumpMonitor::log_state_debug() const {
  LOG_D(
      "%s state: raw_running=%s logical_running=%s effective_enabled=%s "
      "manual_enabled=%s fault_latched=%s short_cycles=%d relay_pin=%d",
      pump_name_.c_str(), current_raw_running() ? "true" : "false",
      logical_running_ ? "true" : "false",
      effective_enabled() ? "true" : "false",
      manual_enabled_ ? "true" : "false", fault_latched_ ? "true" : "false",
      started_ ? short_cycle_count_value_->get() : 0, relay_pin_);
}

bool VacuflushPumpMonitor::to_json(JsonObject& root) {
  root["manual_enabled"] = manual_enabled_;
  root["max_run_time_s"] = max_run_time_s_;
  root["min_stop_time_s"] = min_stop_time_s_;
  root["min_idle_time_s"] = min_idle_time_s_;
  root["min_flush_time_s"] = min_flush_time_s_;
  root["max_consecutive_short_cycles"] = max_consecutive_short_cycles_;
  root["max_cycles_in_window"] = max_cycles_in_window_;
  root["max_flushes_in_window"] = max_flushes_in_window_;
  root["short_cycle_alert_severity"] = short_cycle_alert_severity_;
  root["running_sk_path"] = running_sk_path_;
  root["enabled_sk_path"] = enabled_state_sk_path_;
  root["last_run_time_sk_path"] = last_run_time_sk_path_;
  root["last_idle_time_sk_path"] = last_idle_time_sk_path_;
  root["disabled_time_sk_path"] = disabled_time_sk_path_;
  root["daily_cycle_count_sk_path"] = daily_cycle_count_sk_path_;
  root["short_cycle_count_sk_path"] = short_cycle_count_sk_path_;
  root["average_run_time_sk_path"] = average_run_time_sk_path_;
  root["average_idle_time_sk_path"] = average_idle_time_sk_path_;
  root["daily_flush_count_sk_path"] = daily_flush_count_sk_path_;
  root["total_flush_count_sk_path"] = total_flush_count_sk_path_;
  root["runtime_notification_sk_path"] = runtime_notification_sk_path_;
  root["cycling_notification_sk_path"] = cycling_notification_sk_path_;
  root["relay_notification_sk_path"] = relay_notification_sk_path_;
  root["cycles_notification_sk_path"] = cycles_notification_sk_path_;
  root["flushes_notification_sk_path"] = flushes_notification_sk_path_;
  return true;
}

bool VacuflushPumpMonitor::from_json(const JsonObject& config) {
  if (!config["manual_enabled"].isNull()) {
    manual_enabled_ = config["manual_enabled"].as<bool>();
    if (manual_enabled_) {
      fault_latched_ = false;
    }
  }
  if (!config["max_run_time_s"].isNull()) {
    max_run_time_s_ =
        clamp_min_float(config["max_run_time_s"].as<float>(), 1.0f);
  }
  if (!config["min_stop_time_s"].isNull()) {
    min_stop_time_s_ =
        clamp_min_float(config["min_stop_time_s"].as<float>(), 0.0f);
  }
  if (!config["min_idle_time_s"].isNull()) {
    min_idle_time_s_ =
        clamp_min_float(config["min_idle_time_s"].as<float>(), 0.0f);
  }
  if (!config["min_flush_time_s"].isNull()) {
    min_flush_time_s_ =
        clamp_min_float(config["min_flush_time_s"].as<float>(), 0.0f);
  }
  if (!config["max_consecutive_short_cycles"].isNull()) {
    max_consecutive_short_cycles_ =
        clamp_min_int(config["max_consecutive_short_cycles"].as<int>(), 0);
  }
  if (!config["max_cycles_in_window"].isNull()) {
    max_cycles_in_window_ =
        clamp_min_int(config["max_cycles_in_window"].as<int>(), 0);
  }
  if (!config["max_flushes_in_window"].isNull()) {
    max_flushes_in_window_ =
        clamp_min_int(config["max_flushes_in_window"].as<int>(), 0);
  }
  if (!config["short_cycle_alert_severity"].isNull()) {
    short_cycle_alert_severity_ =
        config["short_cycle_alert_severity"].as<String>();
    short_cycle_alert_severity_.toLowerCase();
    if (short_cycle_alert_severity_ != "warn" &&
        short_cycle_alert_severity_ != "alert") {
      short_cycle_alert_severity_ = kDefaultShortCycleAlertSeverity;
    }
  }
  if (!config["running_sk_path"].isNull()) {
    running_sk_path_ = config["running_sk_path"].as<String>();
  }
  if (!config["enabled_sk_path"].isNull()) {
    enabled_state_sk_path_ = config["enabled_sk_path"].as<String>();
  }
  if (!config["last_run_time_sk_path"].isNull()) {
    last_run_time_sk_path_ = config["last_run_time_sk_path"].as<String>();
  }
  if (!config["last_idle_time_sk_path"].isNull()) {
    last_idle_time_sk_path_ = config["last_idle_time_sk_path"].as<String>();
  }
  if (!config["disabled_time_sk_path"].isNull()) {
    disabled_time_sk_path_ = config["disabled_time_sk_path"].as<String>();
  }
  if (!config["daily_cycle_count_sk_path"].isNull()) {
    daily_cycle_count_sk_path_ = config["daily_cycle_count_sk_path"].as<String>();
  }
  if (!config["short_cycle_count_sk_path"].isNull()) {
    short_cycle_count_sk_path_ = config["short_cycle_count_sk_path"].as<String>();
  }
  if (!config["average_run_time_sk_path"].isNull()) {
    average_run_time_sk_path_ = config["average_run_time_sk_path"].as<String>();
  }
  if (!config["average_idle_time_sk_path"].isNull()) {
    average_idle_time_sk_path_ = config["average_idle_time_sk_path"].as<String>();
  }
  if (!config["daily_flush_count_sk_path"].isNull()) {
    daily_flush_count_sk_path_ = config["daily_flush_count_sk_path"].as<String>();
  }
  if (!config["total_flush_count_sk_path"].isNull()) {
    total_flush_count_sk_path_ = config["total_flush_count_sk_path"].as<String>();
  }
  if (!config["runtime_notification_sk_path"].isNull()) {
    runtime_notification_sk_path_ =
        config["runtime_notification_sk_path"].as<String>();
  }
  if (!config["cycling_notification_sk_path"].isNull()) {
    cycling_notification_sk_path_ =
        config["cycling_notification_sk_path"].as<String>();
  }
  if (!config["relay_notification_sk_path"].isNull()) {
    relay_notification_sk_path_ =
        config["relay_notification_sk_path"].as<String>();
  }
  if (!config["cycles_notification_sk_path"].isNull()) {
    cycles_notification_sk_path_ =
        config["cycles_notification_sk_path"].as<String>();
  }
  if (!config["flushes_notification_sk_path"].isNull()) {
    flushes_notification_sk_path_ =
        config["flushes_notification_sk_path"].as<String>();
  }
  if (!config["daily_stats_day_key"].isNull() &&
      daily_stats_day_key_ != nullptr && daily_stats_day_key_->get() < 0) {
    *daily_stats_day_key_ = config["daily_stats_day_key"].as<int>();
  }
  if (started_) {
    *enabled_state_ = effective_enabled();

    if (effective_enabled()) {
      disabled_since_ms_ = 0;
      *disabled_time_ = 0.0f;
    } else if (disabled_since_ms_ == 0) {
      disabled_since_ms_ = millis();
      last_disabled_report_ms_ = 0;
    }

    republish_all_outputs();
  }

  return true;
}

unsigned long VacuflushPumpMonitor::max_run_time_ms() const {
  return static_cast<unsigned long>(max_run_time_s_ * 1000.0f);
}

unsigned long VacuflushPumpMonitor::min_stop_time_ms() const {
  return static_cast<unsigned long>(min_stop_time_s_ * 1000.0f);
}

unsigned long VacuflushPumpMonitor::min_idle_time_ms() const {
  return static_cast<unsigned long>(min_idle_time_s_ * 1000.0f);
}

unsigned long VacuflushPumpMonitor::min_flush_time_ms() const {
  return static_cast<unsigned long>(min_flush_time_s_ * 1000.0f);
}

unsigned long VacuflushPumpMonitor::cycle_window_duration_ms() const {
  return kCycleWindowDurationMs;
}

bool VacuflushPumpMonitor::short_cycle_should_lockout() const {
  return short_cycle_alert_severity_ == "alert";
}

void VacuflushPumpMonitor::trigger_runtime_fault(const String& message) {
  fault_latched_ = true;
  disabled_since_ms_ = millis();
  last_disabled_report_ms_ = 0;
  *enabled_state_ = false;
  set_notification(runtime_notification_, active_runtime_alert_message_,
                   "alarm", message, true);
}

void VacuflushPumpMonitor::trigger_cycling_fault(const String& message) {
  if (short_cycle_should_lockout()) {
    fault_latched_ = true;
    disabled_since_ms_ = millis();
    last_disabled_report_ms_ = 0;
    *enabled_state_ = false;
  }

  set_notification(cycling_notification_, active_cycling_alert_message_,
                   short_cycle_should_lockout() ? "alert" : "warn", message,
                   false);
}

void VacuflushPumpMonitor::trigger_cycles_fault(const String& message) {
  fault_latched_ = true;
  disabled_since_ms_ = millis();
  last_disabled_report_ms_ = 0;
  *enabled_state_ = false;
  set_notification(cycles_notification_, active_cycles_alert_message_, "alarm",
                   message, true);
}

void VacuflushPumpMonitor::trigger_flushes_fault(const String& message) {
  fault_latched_ = true;
  disabled_since_ms_ = millis();
  last_disabled_report_ms_ = 0;
  *enabled_state_ = false;
  set_notification(flushes_notification_, active_flushes_alert_message_,
                   "alarm", message, true);
}

void VacuflushPumpMonitor::reset_cycle_window(unsigned long now) {
  cycle_window_epoch_ms_ = now;
  cycle_window_count_ = 0;
  cycle_window_flush_count_ = 0;
}

void VacuflushPumpMonitor::update_cycle_window_housekeeping(unsigned long now) {
  if (last_cycle_window_housekeeping_ms_ == 0) {
    last_cycle_window_housekeeping_ms_ = now;
  }

  if (now - last_cycle_window_housekeeping_ms_ <
      kCycleWindowHousekeepingIntervalMs) {
    return;
  }

  last_cycle_window_housekeeping_ms_ = now;

  if (cycle_window_epoch_ms_ == 0 ||
      now - cycle_window_epoch_ms_ > cycle_window_duration_ms()) {
    reset_cycle_window(now);
  }
}

void VacuflushPumpMonitor::handle_running_start(unsigned long now) {
  if (idle_timer_active_) {
    const unsigned long idle_duration = now - idle_start_ms_;
    pending_cycle_idle_duration_ms_ = idle_duration;
    *last_idle_time_ = static_cast<float>(idle_duration) / 1000.0f;
    if (idle_duration <= kMaxIdleTimeForAverageMs) {
      cumulative_idle_time_ms_ += idle_duration;
      idle_sample_count_++;
      *average_idle_time_ = static_cast<float>(cumulative_idle_time_ms_) /
                            static_cast<float>(idle_sample_count_) / 1000.0f;
    }

    if (idle_duration < min_idle_time_ms()) {
      ++(*short_cycle_count_value_);
      if (short_cycle_count_value_->get() > max_consecutive_short_cycles_ &&
          effective_enabled()) {
        trigger_cycling_fault(
            short_cycle_should_lockout()
                ? "Pump exceeded max consecutive short cycles and was auto-disabled"
                : "Pump exceeded max consecutive short cycle threshold");
      }
    } else {
      *short_cycle_count_value_ = 0;
      if (!fault_latched_ && !active_cycling_alert_message_.isEmpty()) {
        clear_notification(cycling_notification_,
                           active_cycling_alert_message_);
      }
    }
  }

  logical_running_ = true;
  cycle_start_ms_ = now;
  stop_candidate_ms_ = 0;
  idle_timer_active_ = false;
  LOG_I("%s: pump ON", pump_name_.c_str());
  *running_state_ = true;
}

void VacuflushPumpMonitor::handle_running_stop(unsigned long now) {
  const unsigned long run_duration = stop_candidate_ms_ - cycle_start_ms_;
  const bool is_flush = pending_cycle_idle_duration_ms_ >= min_idle_time_ms() &&
                        run_duration >= min_flush_time_ms();
  update_daily_cycle_boundary(time(nullptr));
  ++(*daily_cycle_count_);
  if (is_flush) {
    ++(*daily_flush_count_);
    ++(*total_flush_count_);
  }

  if (cycle_window_epoch_ms_ == 0) {
    cycle_window_epoch_ms_ = now;
  }
  cycle_window_count_++;
  if (is_flush) {
    cycle_window_flush_count_++;
  }

  if (cycle_window_count_ > max_cycles_in_window_ && effective_enabled()) {
    trigger_cycles_fault(
        "Pump exceeded max cycles in last 60 minutes and was auto-disabled");
    reset_cycle_window(now);
  } else if (cycle_window_flush_count_ > max_flushes_in_window_ &&
             effective_enabled()) {
    trigger_flushes_fault(
        "Pump exceeded max flushes in last 60 minutes and was auto-disabled");
    reset_cycle_window(now);
  }

  completed_cycle_count_++;
  cumulative_run_time_ms_ += run_duration;

  logical_running_ = false;
  stop_candidate_ms_ = 0;
  cycle_start_ms_ = 0;
  pending_cycle_idle_duration_ms_ = 0;
  idle_start_ms_ = now;
  idle_timer_active_ = true;

  LOG_I("%s: pump OFF", pump_name_.c_str());
  *last_run_time_ = static_cast<float>(run_duration) / 1000.0f;
  *average_run_time_ = static_cast<float>(cumulative_run_time_ms_) /
                       static_cast<float>(completed_cycle_count_) / 1000.0f;
  *running_state_ = false;
}

void VacuflushPumpMonitor::update_disabled_state(unsigned long now) {
  if (effective_enabled()) {
    if (disabled_since_ms_ != 0) {
      disabled_since_ms_ = 0;
      last_disabled_report_ms_ = 0;
      *disabled_time_ = 0.0f;
    }
    return;
  }

  if (disabled_since_ms_ == 0) {
    disabled_since_ms_ = now;
  }

  if (last_disabled_report_ms_ == 0 ||
      now - last_disabled_report_ms_ >= kDisabledReportIntervalMs) {
    *disabled_time_ = static_cast<float>(now - disabled_since_ms_) / 1000.0f;
    last_disabled_report_ms_ = now;
  }

  if (current_raw_running() && now - disabled_since_ms_ >= kDisabledAlertDelayMs) {
    set_notification(relay_notification_, active_relay_alert_message_, "alarm",
                     "Pump still reports running while disabled; check relay or wiring",
                     true);
  } else if (!current_raw_running() && !active_relay_alert_message_.isEmpty()) {
    clear_notification(relay_notification_, active_relay_alert_message_);
  }
}

void VacuflushPumpMonitor::update_short_cycle_window(unsigned long now) {
  if (!idle_timer_active_ || logical_running_) {
    return;
  }

  if (short_cycle_count_value_->get() != 0 &&
      now - idle_start_ms_ >= min_idle_time_ms()) {
    *short_cycle_count_value_ = 0;
    if (!fault_latched_ && !active_cycling_alert_message_.isEmpty()) {
      clear_notification(cycling_notification_,
                         active_cycling_alert_message_);
    }
  }
}

void VacuflushPumpMonitor::evaluate() {
  if (!started_) {
    return;
  }

  const unsigned long now = millis();
  update_daily_cycle_boundary(time(nullptr));
  const bool raw_running = current_raw_running();

  if (raw_running) {
    stop_candidate_ms_ = 0;

    if (!logical_running_) {
      handle_running_start(now);
    }
  } else if (logical_running_ && stop_candidate_ms_ == 0) {
    stop_candidate_ms_ = now;
  }

  if (logical_running_ && !raw_running && stop_candidate_ms_ != 0 &&
      now - stop_candidate_ms_ >= min_stop_time_ms()) {
    handle_running_stop(now);
  }

  if (logical_running_ && effective_enabled() &&
      now - cycle_start_ms_ > max_run_time_ms()) {
    trigger_runtime_fault("Pump exceeded max run time and was auto-disabled");
  }

  update_short_cycle_window(now);
  update_cycle_window_housekeeping(now);
  update_disabled_state(now);
}

const String ConfigSchema(const VacuflushPumpMonitor& obj) {
  return R"json({
    "type":"object",
    "properties":{
      "manual_enabled":{"title":"Enabled","description":"Manual enable setting for this pump. When false, the pump stays disabled until re-enabled from the UI or a Signal K PUT.","type":"boolean"},
      "max_run_time_s":{"title":"Max Run Time (s)","description":"Maximum number of seconds the pump may run continuously before a runtime fault is raised and the pump is auto-disabled.","type":"number"},
      "min_stop_time_s":{"title":"Min Stop Time (s)","description":"Minimum number of seconds the run input must stay off before the pump is considered stopped.","type":"number"},
      "min_idle_time_s":{"title":"Min Idle Time (s)","description":"Idle time threshold used to clear the consecutive short-cycle counter after the pump has remained off long enough.","type":"number"},
      "min_flush_time_s":{"title":"Min Flush Time (s)","description":"Minimum completed run duration required, together with enough preceding idle time, for a cycle to count as a flush.","type":"number"},
      "max_consecutive_short_cycles":{"title":"Max Consecutive Short Cycles","description":"Number of consecutive short cycles allowed before the short-cycle alert logic trips.","type":"integer"},
      "max_cycles_in_window":{"title":"Max Cycles In Last 60 Min","description":"Maximum completed pump cycles allowed within the current 60-minute window before the pump is auto-disabled and an alert is raised.","type":"integer"},
      "max_flushes_in_window":{"title":"Max Flushes In Last 60 Min","description":"Maximum detected flushes allowed within the current 60-minute window before the pump is auto-disabled and an alert is raised.","type":"integer"},
      "short_cycle_alert_severity":{"title":"Short Cycle Alert Severity","description":"Choose whether too many short cycles should only warn or should raise an alert that also auto-disables the pump.","type":"string","enum":["warn","alert"]},
      "running_sk_path":{"title":"Running Path","description":"Signal K path used to report whether the pump is currently running.","type":"string"},
      "enabled_sk_path":{"title":"Enabled State Path","description":"Signal K path used to report and accept PUT control for the pump enabled state.","type":"string"},
      "last_run_time_sk_path":{"title":"Last Run Time Path","description":"Signal K path used to report the duration of the most recently completed pump run cycle.","type":"string"},
      "last_idle_time_sk_path":{"title":"Last Idle Time Path","description":"Signal K path used to report the idle time measured before the current or most recent run cycle.","type":"string"},
      "disabled_time_sk_path":{"title":"Disabled Time Path","description":"Signal K path used to report how long the pump has continuously remained disabled.","type":"string"},
      "daily_cycle_count_sk_path":{"title":"Daily Cycle Count Path","description":"Signal K path used to report the number of completed pump cycles since the most recent midnight boundary.","type":"string"},
      "short_cycle_count_sk_path":{"title":"Short Cycle Count Path","description":"Signal K path used to report the current consecutive short-cycle count.","type":"string"},
      "average_run_time_sk_path":{"title":"Average Run Time Path","description":"Signal K path used to report the average completed cycle run duration since boot.","type":"string"},
      "average_idle_time_sk_path":{"title":"Average Idle Time Path","description":"Signal K path used to report the average idle duration between cycles since boot.","type":"string"},
      "daily_flush_count_sk_path":{"title":"Daily Flush Count Path","description":"Signal K path used to report the number of detected flushes since the most recent midnight boundary.","type":"string"},
      "total_flush_count_sk_path":{"title":"Total Flush Count Path","description":"Signal K path used to report the total number of detected flushes retained across reboot.","type":"string"},
      "runtime_notification_sk_path":{"title":"Runtime Notification Path","description":"Signal K notification path used when the pump exceeds its configured maximum run time.","type":"string"},
      "cycling_notification_sk_path":{"title":"Cycling Notification Path","description":"Signal K notification path used for short-cycle warning or alert conditions.","type":"string"},
      "relay_notification_sk_path":{"title":"Relay Notification Path","description":"Signal K notification path used when the pump still reports running while disabled, indicating a relay or wiring fault.","type":"string"},
      "cycles_notification_sk_path":{"title":"Cycles Notification Path","description":"Signal K notification path used when the completed cycle count exceeds the configured 60-minute limit.","type":"string"},
      "flushes_notification_sk_path":{"title":"Flushes Notification Path","description":"Signal K notification path used when the detected flush count exceeds the configured 60-minute limit.","type":"string"}
    }
  })json";
}

}  // namespace app
