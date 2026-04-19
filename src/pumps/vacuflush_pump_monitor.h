#pragma once

#include <Arduino.h>

#include <memory>
#include <time.h>

#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/system/saveable.h"
#include "sensesp/transforms/lambda_transform.h"
#include "signalk/mutable_bool_sk_put_request_listener.h"
#include "transforms/repeat_mutable.h"

namespace app {

/**
 * @brief SensESP-style "complex component" that monitors and protects one VacuFlush pump.
 *
 * VacuflushPumpMonitor owns the input, derived state, counters, fault logic,
 * relay control, and default Signal K outputs for a single pump. It exposes
 * its key values as producers so other code can subscribe to them, while also
 * providing a convenience `begin()` method that wires the default Signal K
 * outputs and PUT listener.
 */
class VacuflushPumpMonitor : public sensesp::FileSystemSaveable {
 public:
  /**
   * @brief Constructs a pump monitor for one named pump role.
   *
   * @param pump_role Role suffix used in default Signal K paths such as
   *   `master` or `guest`.
   * @param pump_name Human-readable pump name for UI labels and metadata.
   * @param config_path SensESP configuration path for this pump monitor.
   * @param sense_pin Digital input pin used to sense pump running state.
   * @param relay_pin GPIO used to drive the normally-closed interrupt relay.
   */
  VacuflushPumpMonitor(const String& pump_role, const String& pump_name,
                       const String& config_path, int sense_pin,
                       int relay_pin);

  /**
   * @brief Wires the monitor's internal producers to their default consumers.
   *
   * @param state_report_interval_seconds Shared repeat interval for running and
   *   enabled-state reporting.
   * @param aggregate_report_interval_seconds Shared repeat interval for counts
   *   and aggregate statistics reporting.
   */
  void begin(float state_report_interval_seconds,
             float aggregate_report_interval_seconds);
  void set_manual_enabled(bool enabled, bool persist = true);
  void log_state_debug() const;
  void onSKServerConnect();

  const String& pump_name() const { return pump_name_; }
  const String& pump_role() const { return pump_role_; }

  sensesp::DigitalInputState* raw_run_input() const { return raw_run_input_; }
  sensesp::BoolProducer* running_producer() const { return running_state_; }
  sensesp::BoolProducer* enabled_state_producer() const {
    return enabled_state_;
  }
  sensesp::FloatProducer* last_run_time_producer() const {
    return last_run_time_;
  }
  sensesp::FloatProducer* last_idle_time_producer() const {
    return last_idle_time_;
  }
  sensesp::FloatProducer* disabled_time_producer() const {
    return disabled_time_;
  }
  sensesp::IntProducer* daily_cycle_count_producer() const {
    return daily_cycle_count_;
  }
  sensesp::IntProducer* short_cycle_count_producer() const {
    return short_cycle_count_value_;
  }
  sensesp::FloatProducer* average_run_time_producer() const {
    return average_run_time_;
  }
  sensesp::FloatProducer* average_idle_time_producer() const {
    return average_idle_time_;
  }
  sensesp::IntProducer* daily_flush_count_producer() const {
    return daily_flush_count_;
  }
  sensesp::PersistingObservableValue<int>* total_flush_count_producer() const {
    return total_flush_count_;
  }
  sensesp::StringProducer* runtime_notification_producer() const {
    return runtime_notification_;
  }
  sensesp::StringProducer* cycling_notification_producer() const {
    return cycling_notification_;
  }
  sensesp::StringProducer* relay_notification_producer() const {
    return relay_notification_;
  }

  bool to_json(JsonObject& root) override;
  bool from_json(const JsonObject& config) override;

 private:
  bool effective_enabled() const { return manual_enabled_ && !fault_latched_; }
  bool current_raw_running() const;
  unsigned long max_run_time_ms() const;
  unsigned long min_stop_time_ms() const;
  unsigned long min_idle_time_ms() const;
  unsigned long min_flush_time_ms() const;
  unsigned long cycle_window_duration_ms() const;
  bool short_cycle_should_lockout() const;

  void initialize_paths();
  String make_pump_path(const String& suffix) const;
  String make_switch_path(const String& suffix) const;
  String make_notification_path(const String& suffix) const;
  void republish_all_outputs();
  void update_daily_cycle_boundary(time_t now_seconds);
  int day_key_from_time(time_t now_seconds) const;

  String make_notification_payload(const char* state, const String& message,
                                   bool audible) const;
  String make_cleared_notification_payload() const;
  sensesp::SKMetadata* make_running_metadata() const;
  sensesp::SKMetadata* make_enabled_state_metadata() const;
  sensesp::SKMetadata* make_last_run_time_metadata() const;
  sensesp::SKMetadata* make_last_idle_time_metadata() const;
  sensesp::SKMetadata* make_disabled_time_metadata() const;
  sensesp::SKMetadata* make_daily_cycle_count_metadata() const;
  sensesp::SKMetadata* make_short_cycle_count_metadata() const;
  sensesp::SKMetadata* make_average_run_time_metadata() const;
  sensesp::SKMetadata* make_average_idle_time_metadata() const;
  sensesp::SKMetadata* make_daily_flush_count_metadata() const;
  sensesp::SKMetadata* make_total_flush_count_metadata() const;
  sensesp::SKMetadata* make_runtime_notification_metadata() const;
  sensesp::SKMetadata* make_cycling_notification_metadata() const;
  sensesp::SKMetadata* make_relay_notification_metadata() const;
  sensesp::SKMetadata* make_cycles_notification_metadata() const;
  sensesp::SKMetadata* make_flushes_notification_metadata() const;

  void set_notification(sensesp::ObservableValue<String>* producer,
                        String& active_message, const char* state,
                        const String& message, bool audible);
  void clear_notification(sensesp::ObservableValue<String>* producer,
                          String& active_message);
  void clear_all_notifications();
  void trigger_runtime_fault(const String& message);
  void trigger_cycling_fault(const String& message);
  void trigger_cycles_fault(const String& message);
  void trigger_flushes_fault(const String& message);
  void reset_cycle_window(unsigned long now);
  void update_cycle_window_housekeeping(unsigned long now);

  void evaluate();
  void handle_running_start(unsigned long now);
  void handle_running_stop(unsigned long now);
  void update_disabled_state(unsigned long now);
  void update_short_cycle_window(unsigned long now);

  String pump_role_;
  String pump_name_;
  int sense_pin_;
  int relay_pin_;
  bool started_ = false;

  bool manual_enabled_;
  bool fault_latched_ = false;

  float max_run_time_s_;
  float min_stop_time_s_;
  float min_idle_time_s_;
  float min_flush_time_s_;
  int max_consecutive_short_cycles_;
  int max_cycles_in_window_;
  int max_flushes_in_window_;
  String short_cycle_alert_severity_;

  String running_sk_path_;
  String enabled_state_sk_path_;
  String last_run_time_sk_path_;
  String last_idle_time_sk_path_;
  String disabled_time_sk_path_;
  String daily_cycle_count_sk_path_;
  String short_cycle_count_sk_path_;
  String average_run_time_sk_path_;
  String average_idle_time_sk_path_;
  String daily_flush_count_sk_path_;
  String total_flush_count_sk_path_;
  String runtime_notification_sk_path_;
  String cycling_notification_sk_path_;
  String relay_notification_sk_path_;
  String cycles_notification_sk_path_;
  String flushes_notification_sk_path_;

  bool logical_running_ = false;
  bool idle_timer_active_ = false;
  unsigned long cycle_start_ms_ = 0;
  unsigned long stop_candidate_ms_ = 0;
  unsigned long idle_start_ms_ = 0;
  unsigned long pending_cycle_idle_duration_ms_ = 0;
  unsigned long cycle_window_epoch_ms_ = 0;
  unsigned long last_cycle_window_housekeeping_ms_ = 0;
  unsigned long disabled_since_ms_ = 0;
  unsigned long last_disabled_report_ms_ = 0;
  int cycle_window_count_ = 0;
  int cycle_window_flush_count_ = 0;
  unsigned long long cumulative_run_time_ms_ = 0;
  unsigned long long cumulative_idle_time_ms_ = 0;
  unsigned long completed_cycle_count_ = 0;
  unsigned long idle_sample_count_ = 0;

  String active_runtime_alert_message_;
  String active_cycling_alert_message_;
  String active_relay_alert_message_;
  String active_cycles_alert_message_;
  String active_flushes_alert_message_;

  sensesp::DigitalInputState* raw_run_input_ = nullptr;
  sensesp::DigitalOutput* relay_output_ = nullptr;
  sensesp::LambdaTransform<bool, bool>* relay_drive_transform_ = nullptr;

  sensesp::ObservableValue<bool>* running_state_ = nullptr;
  sensesp::ObservableValue<bool>* enabled_state_ = nullptr;
  sensesp::ObservableValue<float>* last_run_time_ = nullptr;
  sensesp::ObservableValue<float>* last_idle_time_ = nullptr;
  sensesp::ObservableValue<float>* disabled_time_ = nullptr;
  sensesp::ObservableValue<int>* daily_cycle_count_ = nullptr;
  sensesp::ObservableValue<int>* short_cycle_count_value_ = nullptr;
  sensesp::ObservableValue<float>* average_run_time_ = nullptr;
  sensesp::ObservableValue<float>* average_idle_time_ = nullptr;
  sensesp::PersistingObservableValue<int>* daily_stats_day_key_ = nullptr;
  sensesp::PersistingObservableValue<int>* daily_flush_count_ = nullptr;
  sensesp::PersistingObservableValue<int>* total_flush_count_ = nullptr;
  sensesp::ObservableValue<String>* runtime_notification_ = nullptr;
  sensesp::ObservableValue<String>* cycling_notification_ = nullptr;
  sensesp::ObservableValue<String>* relay_notification_ = nullptr;
  sensesp::ObservableValue<String>* cycles_notification_ = nullptr;
  sensesp::ObservableValue<String>* flushes_notification_ = nullptr;

  reactesp::RepeatEvent* evaluation_event_ = nullptr;
};

const String ConfigSchema(const VacuflushPumpMonitor& obj);

inline bool ConfigRequiresRestart(const VacuflushPumpMonitor& obj) {
  return false;
}

}  // namespace app
