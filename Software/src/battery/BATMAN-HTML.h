#ifndef _BATMAN_HTML_H
#define _BATMAN_HTML_H

#include "../datalayer/datalayer.h"
#include "src/devboard/webserver/BatteryHtmlRenderer.h"

// Forward declaration
class BatmanBattery;

class BatmanHtmlRenderer : public BatteryHtmlRenderer {
 public:
  BatmanHtmlRenderer(BatmanBattery& b) : batt(b) {}
  String get_status_html();

 private:
  BatmanBattery& batt;
};

#endif 