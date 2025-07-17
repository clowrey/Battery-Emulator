#include "BATMAN-HTML.h"
#include "BATMAN-BATTERY.h"
#include "../datalayer/datalayer.h"

String BatmanHtmlRenderer::get_status_html() {
  String content = "";
  battery_specific_status(&content);
  return content;
}

void BatmanHtmlRenderer::battery_specific_status(String* content) {
  // This function displays Batman-specific information on the web interface
  // Data is accessed through the global datalayer structure
  
  content->concat("<h4>Batman IC Status</h4>");
  content->concat("<table>");
  
  // Display cell voltage statistics
  content->concat("<tr><td>Cells Present:</td><td>");
  content->concat(String(datalayer.battery.info.number_of_cells));
  content->concat("</td></tr>");
  
  content->concat("<tr><td>Max Cell Voltage:</td><td>");
  content->concat(String(datalayer.battery.status.cell_max_voltage_mV));
  content->concat(" mV</td></tr>");
  
  content->concat("<tr><td>Min Cell Voltage:</td><td>");
  content->concat(String(datalayer.battery.status.cell_min_voltage_mV));
  content->concat(" mV</td></tr>");
  
  content->concat("<tr><td>Cell Voltage Delta:</td><td>");
  content->concat(String(datalayer.battery.status.cell_max_voltage_mV - datalayer.battery.status.cell_min_voltage_mV));
  content->concat(" mV</td></tr>");
  
  // Display temperature information
  content->concat("<tr><td>Max Temperature:</td><td>");
  content->concat(String(datalayer.battery.status.temperature_max_dC / 10.0, 1));
  content->concat(" °C</td></tr>");
  
  content->concat("<tr><td>Min Temperature:</td><td>");
  content->concat(String(datalayer.battery.status.temperature_min_dC / 10.0, 1));
  content->concat(" °C</td></tr>");
  
  // Display power and current information
  content->concat("<tr><td>Pack Current:</td><td>");
  content->concat(String(datalayer.battery.status.current_dA / 10.0, 1));
  content->concat(" A</td></tr>");
  
  content->concat("<tr><td>Pack Power:</td><td>");
  content->concat(String(datalayer.battery.status.active_power_W));
  content->concat(" W</td></tr>");
  
  // Display balancing information
  content->concat("<tr><td>Balancing Status:</td><td>");
  if (datalayer.battery.status.bms_status == ACTIVE) {
    content->concat("Active");
  } else {
    content->concat("Inactive/Fault");
  }
  content->concat("</td></tr>");
  
  content->concat("</table>");
  
  // Display Batman IC specific information
  content->concat("<h4>Batman IC Communication</h4>");
  content->concat("<table>");
  
  content->concat("<tr><td>Interface:</td><td>");
  content->concat(battery_.interface_name());
  content->concat("</td></tr>");
  content->concat("<tr><td>Communication:</td><td>SPI @ 1MHz</td></tr>");
  content->concat("<tr><td>Current Sensor:</td><td>AS8510 Coulomb Counter</td></tr>");
  
  content->concat("</table>");
  
  // Display safety limits
  content->concat("<h4>Safety Limits</h4>");
  content->concat("<table>");
  
  content->concat("<tr><td>Max Cell Voltage Limit:</td><td>4.25V</td></tr>");
  content->concat("<tr><td>Min Cell Voltage Limit:</td><td>2.7V</td></tr>");
  content->concat("<tr><td>Max Pack Voltage:</td><td>420V</td></tr>");
  content->concat("<tr><td>Min Pack Voltage:</td><td>280V</td></tr>");
  content->concat("<tr><td>Balance Hysteresis:</td><td>20mV</td></tr>");
  content->concat("<tr><td>Manual Balancing:</td><td>");
  content->concat(battery_.supports_manual_balancing() ? "Supported" : "Not Supported");
  content->concat("</td></tr>");
  content->concat("<tr><td>Real BMS Status:</td><td>");
  content->concat(battery_.supports_real_BMS_status() ? "Supported" : "Not Supported");
  content->concat("</td></tr>");
  
  content->concat("</table>");
  
  // Display individual cell voltages if available
  if (datalayer.battery.info.number_of_cells > 0) {
    content->concat("<h4>Individual Cell Voltages</h4>");
    content->concat("<div style='max-height: 300px; overflow-y: auto;'>");
    content->concat("<table style='font-size: 12px;'>");
    content->concat("<tr><th>Cell</th><th>Voltage (mV)</th><th>Balancing</th></tr>");
    
    for (int i = 0; i < datalayer.battery.info.number_of_cells && i < MAX_AMOUNT_CELLS; i++) {
      uint16_t voltage = datalayer.battery.status.cell_voltages_mV[i];
      bool balancing = datalayer.battery.status.cell_balancing_status[i];
      
      // All cells in the array are valid (sequential storage)
      content->concat("<tr>");
      content->concat("<td>");
      content->concat(String(i + 1));
      content->concat("</td>");
      content->concat("<td>");
      content->concat(String(voltage));
      content->concat("</td>");
      content->concat("<td>");
      content->concat(balancing ? "Yes" : "No");
      content->concat("</td>");
      content->concat("</tr>");
    }
    
    content->concat("</table>");
    content->concat("</div>");
  }
} 