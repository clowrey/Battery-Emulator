#include "BATMAN-HTML.h"
#include "../include.h"
#include "BATMAN-BATTERY.h"

String BatmanHtmlRenderer::get_status_html() {
  String content;

#ifdef BATMAN_BATTERY
  // Display Tesla Model 3 Batman IC specific information
  content += "<h4>Tesla Model 3 Batman IC Cell Monitor</h4>";
  
  // Basic voltage information
  content += "<h4>Min Cell Voltage: " + String(batt.getMinVoltage(), 1) + " mV (Cell " + String(batt.getMinCell()) + ")</h4>";
  content += "<h4>Max Cell Voltage: " + String(batt.getMaxVoltage(), 1) + " mV (Cell " + String(batt.getMaxCell()) + ")</h4>";
  content += "<h4>Voltage Delta: " + String(batt.getMaxVoltage() - batt.getMinVoltage(), 1) + " mV</h4>";
  
  // Balancing information
  auto balancingInfo = batt.getBalancingInfo();
  content += "<h4>Total Cells Detected: " + String(balancingInfo.totalCells) + "</h4>";
  content += "<h4>Cells Currently Balancing: " + String(balancingInfo.balancingCells) + "</h4>";
  
  // Display which cells are currently being balanced (if any)
  if (balancingInfo.balancingCells > 0) {
    content += "<h4>Cells Being Balanced: ";
    for (int i = 0; i < balancingInfo.balancingCells; i++) {
      content += String(balancingInfo.balancingCellNumbers[i]);
      if (i < balancingInfo.balancingCells - 1) {
        content += ", ";
      }
    }
    content += "</h4>";
  }
  
  // Additional Batman IC specific information
  content += "<h4>BMB Communication Status: ";
  if (batt.getBmbTimeout()) {
    content += "<span style='color: #FF0000; font-weight: bold;'>TIMEOUT</span></h4>";
  } else {
    content += "<span style='color: #00FF00; font-weight: bold;'>OK</span></h4>";
  }
  
  content += "<h4>Active BMB Count: " + String(batt.getActiveBmbCount()) + " / 8</h4>";
  
  content += "<h4>Balancing Control: ";
  if (batt.isBalancingEnabled()) {
    content += "<span style='color: #00FF00; font-weight: bold;'>ENABLED</span>";
  } else {
    content += "<span style='color: #FF6600; font-weight: bold;'>DISABLED</span>";
  }
  content += " <button onclick='enableBatmanBalance()' style='margin-left: 10px; background-color: #4CAF50; color: white; border: none; padding: 5px 10px; border-radius: 3px; cursor: pointer;'>Enable</button>";
  content += " <button onclick='disableBatmanBalance()' style='margin-left: 5px; background-color: #f44336; color: white; border: none; padding: 5px 10px; border-radius: 3px; cursor: pointer;'>Disable</button></h4>";

  content += "<h4>Balancing Hysteresis: <span id='balHysteresis'>" + String(datalayer.battery.settings.balancing_hysteresis_mV) + " mV</span> <button onclick='editBalancingHysteresis()' style='margin-left: 10px; background-color: #2196F3; color: white; border: none; padding: 5px 10px; border-radius: 3px; cursor: pointer;'>Edit</button></h4>";

  content += "<h4>Balancing Status: ";
  if (batt.getBalancingStatus()) {
    content += "<span style='color: #FF0000; font-weight: bold;'>ACTIVE (Cells Need Balancing)</span></h4>";
  } else {
    content += "<span style='color: #00FF00; font-weight: bold;'>INACTIVE (Cells Balanced)</span></h4>";
  }
  
  content += "<h4>Balance Phase: ";
  uint8_t phase = batt.getBalancePhase();
  switch (phase) {
    case 0:
      content += "Measurement Only (No Balancing)</h4>";
      break;
    case 1:
      content += "Even Cells</h4>";
      break;
    case 2:
      content += "Odd Cells</h4>";
      break;
    default:
      content += "Unknown</h4>";
  }

  // JavaScript for Batman balance control with separate enable/disable functions
  content += "<script>";
  content += "function editComplete(){if(this.status==200){window.location.reload();}}";
  content += "function editError(){alert('Failed to update balancing setting');}";
  content += "function enableBatmanBalance(){if(confirm('Enable Batman IC cell balancing? This will start the 3-phase balancing cycle (measurement, even cells, odd cells).')){var xhr=new XMLHttpRequest();xhr.onload=editComplete;xhr.onerror=editError;xhr.open('GET','/BatmanBalAct?value=1',true);xhr.send();}}";
  content += "function disableBatmanBalance(){if(confirm('Disable Batman IC cell balancing? This will stop balancing and only perform voltage measurements.')){var xhr=new XMLHttpRequest();xhr.onload=editComplete;xhr.onerror=editError;xhr.open('GET','/BatmanBalAct?value=0',true);xhr.send();}}";
  content += "function editBalancingHysteresis(){var newValue=prompt('Enter balancing hysteresis (1-100 mV):', '" + String(datalayer.battery.settings.balancing_hysteresis_mV) + "');if(newValue!==null&&newValue>=1&&newValue<=100){var xhr=new XMLHttpRequest();xhr.onload=editComplete;xhr.onerror=editError;xhr.open('GET','/updateBalancingHysteresis?value='+newValue,true);xhr.send();}else if(newValue!==null){alert('Please enter a value between 1 and 100 mV');}}";
  content += "</script>";

#else
  content += "<h4>Batman IC functionality not compiled in</h4>";
#endif

  return content;
} 