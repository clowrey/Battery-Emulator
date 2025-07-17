#ifndef BATMAN_HTML_H
#define BATMAN_HTML_H

#include "../devboard/webserver/BatteryHtmlRenderer.h"
#include "../system_settings.h"

class BatmanBattery;  // Forward declaration

class BatmanHtmlRenderer : public BatteryHtmlRenderer {
 public:
  explicit BatmanHtmlRenderer(const BatmanBattery& battery) : battery_(battery) {}

  String get_status_html() override;
  void battery_specific_status(String* content);

 private:
  const BatmanBattery& battery_;
};

#endif  // BATMAN_HTML_H 