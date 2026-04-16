#include <Arduino.h>

#include <memory>

#include <ArduinoOTA.h>
#include <WiFi.h>

#if __has_include("device_config.h")
#include "device_config.h"
#else
#error "Missing device_config.h. Copy include/device_config.example.h to include/device_config.h and fill in your local settings before building."
#endif

#include "Debug.h"
#include "pumps/vacuflush_pump_monitor.h"
#include "sensesp/signalk/signalk_ws_client.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "signalk/signalk_time_sync.h"
#include "system/heartbeat_reporter.h"
#include "system/halmet_const.h"
#include "system/halmet_serial.h"
#include "system/ui_counter.h"


using namespace sensesp;
using namespace halmet;
using namespace app;

namespace {

constexpr int kMasterPumpRelayPin = 16;
constexpr int kGuestPumpRelayPin = 17;

constexpr const char* kMasterPumpRole = "master";
constexpr const char* kMasterPumpName = "Master Head Pump";
constexpr const char* kMasterPumpConfigPath = "/pumps/master";

constexpr const char* kGuestPumpRole = "guest";
constexpr const char* kGuestPumpName = "Guest Head Pump";
constexpr const char* kGuestPumpConfigPath = "/pumps/guest";

constexpr unsigned int kPumpStateLogIntervalMs = 2000;
constexpr unsigned int kNetworkStatusLogIntervalMs = 5000;
constexpr float kDefaultPumpStateReportIntervalSeconds = 2.0f;
constexpr float kDefaultPumpAggregateReportIntervalSeconds = 30.0f;
constexpr const char* kPumpStateReportIntervalConfigPath =
    "/system/pump_state_report_interval";
constexpr const char* kPumpAggregateReportIntervalConfigPath =
    "/system/pump_aggregate_report_interval_seconds";
constexpr const char* kHeartbeatConfigPath = "/system/heartbeat";
constexpr float kDefaultHeartbeatIntervalSeconds = 10.0f;
constexpr const char* kHeartbeatSignalKPath =
    "electrical.pumps.sanitation.vacuum.monitor.heartbeat";

std::shared_ptr<VacuflushPumpMonitor> master_pump;
std::shared_ptr<VacuflushPumpMonitor> guest_pump;
std::shared_ptr<HeartbeatReporter> heartbeat_reporter;
std::shared_ptr<PersistingObservableValue<float>> pump_state_report_interval_s;
std::shared_ptr<PersistingObservableValue<float>>
    pump_aggregate_report_interval_s;
std::shared_ptr<SignalKTimeSync> signalk_time_sync;

}  // namespace

void configure_ship_timezone() {
  setenv("TZ", kShipTimeZonePosix, 1);
  tzset();
  LOG_I("Time sync: ship timezone set to %s (%s)", kShipTimeZoneName,
        kShipTimeZonePosix);
}


void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  Serial.begin(115200);
  debug_setup();

  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname(kDeviceHostname)
                    ->set_wifi_client(kBoatWifiSsid, kBoatWifiPassword)
                    ->set_wifi_access_point(kDeviceHostname, kBoatWifiPassword)
                    ->set_sk_server(kSignalKServerHost, kSignalKServerPort)
                    ->get_app();

  LOG_I("Signal K: direct server mode enabled host=%s port=%u use_mdns=false",
        kSignalKServerHost, kSignalKServerPort);

  configure_ship_timezone();

  signalk_time_sync = std::make_shared<SignalKTimeSync>();
  signalk_time_sync->begin();

  master_pump = std::make_shared<VacuflushPumpMonitor>(
      kMasterPumpRole, kMasterPumpName, kMasterPumpConfigPath,
      kDigitalInputPin1, kMasterPumpRelayPin);

  guest_pump = std::make_shared<VacuflushPumpMonitor>(
      kGuestPumpRole, kGuestPumpName, kGuestPumpConfigPath,
      kDigitalInputPin2, kGuestPumpRelayPin);

  pump_state_report_interval_s = std::make_shared<PersistingObservableValue<float>>(
      kDefaultPumpStateReportIntervalSeconds, kPumpStateReportIntervalConfigPath);

  UICounter uiCounter;

  ConfigItem(pump_state_report_interval_s)
      ->set_title("Pump State Report Interval (s)")
      ->set_description(
          "How often the running and enabled states are re-sent to Signal K for both pumps.")
      ->set_sort_order(uiCounter.nextValue());

  pump_aggregate_report_interval_s =
      std::make_shared<PersistingObservableValue<float>>(
          kDefaultPumpAggregateReportIntervalSeconds,
          kPumpAggregateReportIntervalConfigPath);
  ConfigItem(pump_aggregate_report_interval_s)
      ->set_title("Pump Aggregate Report Interval (s)")
      ->set_description(
          "How often cycle counts and average cycle statistics are re-sent to Signal K for both pumps.")
      ->set_sort_order(uiCounter.nextValue());

  master_pump->begin(pump_state_report_interval_s->get(),
                     pump_aggregate_report_interval_s->get());
  guest_pump->begin(pump_state_report_interval_s->get(),
                    pump_aggregate_report_interval_s->get());

  ConfigItem(master_pump)
      ->set_title("Master Head Pump")
      ->set_sort_order(uiCounter.nextValue());
  ConfigItem(guest_pump)
      ->set_title("Guest Head Pump")
      ->set_sort_order(uiCounter.nextValue());

  sensesp_app->get_ws_client()->connect_to(
      new LambdaConsumer<SKWSConnectionState>([](SKWSConnectionState state) {
        if (state == SKWSConnectionState::kSKWSConnected) {
          LOG_I("Signal K: websocket connected, emitting full pump state");
          master_pump->onSKServerConnect();
          guest_pump->onSKServerConnect();
        }
      }));

  heartbeat_reporter = std::make_shared<HeartbeatReporter>(
      kHeartbeatConfigPath, kHeartbeatSignalKPath,
      kDefaultHeartbeatIntervalSeconds);
  ConfigItem(heartbeat_reporter)
      ->set_title("Device Heartbeat")
      ->set_sort_order(uiCounter.nextValue());

  event_loop()->onDelay(0, []() {
    ArduinoOTA.setHostname(kDeviceHostname);
    ArduinoOTA.setPassword(kBoatWifiPassword);
    ArduinoOTA.onStart([]() { LOG_W("OTA: starting update"); });
    ArduinoOTA.onEnd([]() { LOG_W("OTA: update complete"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      LOG_I("OTA: progress %u%%", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      LOG_E("OTA: error[%u]", error);
      if (error == OTA_AUTH_ERROR) {
        LOG_E("OTA: auth failed");
      } else if (error == OTA_BEGIN_ERROR) {
        LOG_E("OTA: begin failed");
      } else if (error == OTA_CONNECT_ERROR) {
        LOG_E("OTA: connect failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        LOG_E("OTA: receive failed");
      } else if (error == OTA_END_ERROR) {
        LOG_E("OTA: end failed");
      }
    });
    ArduinoOTA.begin();
    event_loop()->onRepeat(20, []() { ArduinoOTA.handle(); });
  });

  event_loop()->onRepeat(kPumpStateLogIntervalMs, []() {
    master_pump->log_state_debug();
    guest_pump->log_state_debug();
  });

  event_loop()->onRepeat(kNetworkStatusLogIntervalMs, []() {
    const wl_status_t wifi_status = WiFi.status();
    const bool connected = wifi_status == WL_CONNECTED;
    const String current_ssid = connected ? WiFi.SSID() : "";
    const bool on_primary_boat_network = connected && current_ssid == kBoatWifiSsid;
    const String ip = connected ? WiFi.localIP().toString() : "0.0.0.0";

    LOG_I(
        "Network: connected=%s status=%d current_ssid='%s' expected_ssid='%s' "
        "primary_match=%s ip=%s",
        connected ? "true" : "false", static_cast<int>(wifi_status),
        current_ssid.c_str(), kBoatWifiSsid,
        on_primary_boat_network ? "true" : "false", ip.c_str());
  });

  LOG_I("Vacuflush monitoring firmware started");
}

void loop() {
  event_loop()->tick();
  debug_loop();
}
