// Fresh, minimal HID bridge: Wi-Fi AP + Web UI + BLE Central (NimBLE) + USB HID pass-through with mapping

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <nvs_flash.h>
#include <nvs.h>

// Try Arduino-ESP32 TinyUSB HID classes (ESP32-S3)
#include <USB.h>
#include <USBHID.h>
#include <vector>

// -------------------- Wi-Fi AP --------------------
static const char* AP_SSID = "HTPC-BLEBoot"; // open AP (no password)
static const char* AP_PASS = "";

// -------------------- Web Server --------------------
AsyncWebServer server(80);

// -------------------- Preferences --------------------
Preferences prefs;

// Button mapping structure (up to 64 custom mappings)
struct ButtonMap {
  bool active = false;           // Is this slot in use?
  char label[32] = "Power";      // User-defined label
  uint8_t reportId = 3;          // Learned Report ID
  uint8_t payloadLen = 1;        // Learned payload length
  uint8_t payload[8] = {0x01, 0, 0, 0, 0, 0, 0, 0}; // Learned button pattern
  uint8_t action = 2;            // 0=USB, 1=GPIO toggle, 2=LED color, 3=passthrough, 4=Keyboard, 5=Media
  uint8_t gpioPin = 2;           // GPIO pin for action 1
  uint8_t gpioMode = 0;          // 0=Toggle, 1=Pull HIGH, 2=Pull LOW (on button release)
  uint16_t ttlMs = 0;            // Time-to-live in ms (0=stay until release, 1-9999=auto-revert)
  uint8_t gpioTtlBehavior = 0;   // 0=Switch to opposite state after TTL, 1=Float pin after TTL
  bool passToUsb = false;        // Also forward to USB
  uint8_t ledR = 0;              // LED Red (0-255)
  uint8_t ledG = 255;            // LED Green (0-255)
  uint8_t ledB = 0;              // LED Blue (0-255)
  bool useHoldTime = false;      // Enable per-button hold time requirement
  uint8_t holdTimeSec = 3;       // Hold time in seconds (1-10)
  bool applyToUsb = false;       // Apply hold time to USB data (single send vs repeat)
  uint16_t keyCode = 0x2A;       // HID key code for keyboard (8-bit scan code)
  uint8_t keyModifier = 0x00;    // Modifier keys (Ctrl=0x01, Shift=0x02, Alt=0x04, Win=0x08)
  uint16_t mediaCode = 0x00E9;   // Consumer control code for media (16-bit usage ID)
};

#define MAX_BUTTON_MAPS 64
ButtonMap buttonMaps[MAX_BUTTON_MAPS];
int activeButtonMaps = 0;

// Button press tracking for minimum hold time
struct ButtonPressState {
  bool isPressed = false;
  unsigned long pressStartTime = 0;
  bool actionTriggered = false;
};
ButtonPressState buttonPressStates[MAX_BUTTON_MAPS];

// TTL timer tracking for auto-revert
struct TTLState {
  bool active = false;
  unsigned long startTime = 0;
  uint8_t mapIndex = 0;
};
TTLState ttlTimers[MAX_BUTTON_MAPS];

// Global state tracking for USB actions
bool gpioStates[MAX_BUTTON_MAPS] = {false};
bool gpioToggleStates[MAX_BUTTON_MAPS] = {false}; // Persistent toggle state (not affected by TTL)
bool usbSentOnce[MAX_BUTTON_MAPS] = {false};

// Learn mode state
static volatile bool learnMode = false;
static int learnMapIndex = -1; // Which map slot is being learned
static unsigned long learnModeStartTime = 0;
static const unsigned long LEARN_TIMEOUT = 10000; // 10 seconds

// Built-in LED (ESP32-S3 DevKit, GPIO48 for WS2812)
#define LED_PIN 48
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// -------------------- USB HID --------------------
// Custom HID class that uses captured descriptor
class CustomHID : public USBHIDDevice {
public:
  static USBHID hid;  // Public static shared HID interface
  
private:
  uint8_t* descriptor;
  uint16_t descriptorLength;

public:
  CustomHID(void) {
    descriptor = nullptr;
    descriptorLength = 0;
  }

  void begin(uint8_t* desc, uint16_t len) {
    descriptor = desc;
    descriptorLength = len;
    
    // Register device and start HID - only once with static flags
    static bool initialized = false;
    if (!initialized && descriptor && descriptorLength > 0) {
      hid.addDevice(this, descriptorLength);
      initialized = true;
      Serial.printf("CustomHID: Device registered with descriptor length %d\n", descriptorLength);
      
      // Now start the HID interface
      hid.begin();
      Serial.println("CustomHID::begin() called, HID interface started");
    }
  }

  // Required by USBHIDDevice - return captured descriptor
  uint16_t _onGetDescriptor(uint8_t* buffer) {
    if (descriptor && descriptorLength > 0) {
      memcpy(buffer, descriptor, descriptorLength);
      Serial.printf("CustomHID::_onGetDescriptor() returning %d bytes\n", descriptorLength);
      return descriptorLength;
    }
    Serial.println("CustomHID::_onGetDescriptor() returning 0 - no descriptor!");
    return 0;
  }

  // Send raw HID report using USBHID.SendReport
  bool send(uint8_t reportID, const uint8_t* data, size_t len) {
    // Check if HID is ready
    if (!hid.ready()) {
      return false;
    }
    
    // Use USBHID::SendReport - expects reportID and data separately
    return hid.SendReport(reportID, data, len);
  }
};

// Define the static member
USBHID CustomHID::hid;

CustomHID customHID;
static uint8_t capturedDescriptor[256];
static uint16_t capturedDescriptorLen = 0;
static bool usbStarted = false;

// -------------------- Persistent HID Descriptor Storage --------------------
bool loadSavedDescriptor() {
  prefs.begin("hid_desc", true); // read-only
  size_t len = prefs.getBytesLength("desc");
  if (len > 0 && len <= 256) {
    capturedDescriptorLen = prefs.getBytes("desc", capturedDescriptor, len);
    prefs.end();
    Serial.printf("Loaded saved HID descriptor from NVS (%d bytes)\n", capturedDescriptorLen);
    return true;
  }
  prefs.end();
  Serial.println("No saved HID descriptor found in NVS");
  return false;
}

bool saveDescriptor(const uint8_t* desc, uint16_t len) {
  if (!desc || len == 0 || len > 256) {
    Serial.println("Invalid descriptor - cannot save");
    return false;
  }
  
  prefs.begin("hid_desc", false); // read-write
  size_t written = prefs.putBytes("desc", desc, len);
  prefs.end();
  
  if (written == len) {
    Serial.printf("Saved HID descriptor to NVS (%d bytes)\n", len);
    return true;
  } else {
    Serial.println("Failed to save HID descriptor to NVS");
    return false;
  }
}

bool deleteSavedDescriptor() {
  prefs.begin("hid_desc", false); // read-write
  bool removed = prefs.remove("desc");
  prefs.end();
  
  if (removed) {
    Serial.println("Deleted saved HID descriptor from NVS");
    capturedDescriptorLen = 0;
    return true;
  } else {
    Serial.println("No saved HID descriptor to delete");
    return false;
  }
}

bool hasSavedDescriptor() {
  prefs.begin("hid_desc", true); // read-only
  size_t len = prefs.getBytesLength("desc");
  prefs.end();
  return (len > 0 && len <= 256);
}

// Track simple keyboard state to release keys
uint8_t prevKeys[6] = {0};
uint8_t prevMod = 0;

// -------------------- BLE (NimBLE) --------------------
NimBLEScan* pScan = nullptr;
NimBLEClient* pClient = nullptr;
NimBLERemoteService* pHID = nullptr;

// Security
static uint32_t passkey = 123456;

// Keep references to input report characteristics and their Report IDs
struct ReportChar {
  NimBLERemoteCharacteristic* chr = nullptr;
  uint8_t reportId = 0; // 0 means no explicit ID
  uint8_t reportType = 0; // 1 = Input (from Report Reference descriptor)
};
std::vector<ReportChar> inputReports;

// Status
String savedAddr = "";
String savedName = "";
String activeAddr = "";
String activeName = "";
volatile bool isConnecting = false;
volatile bool isConnected = false;
String hidMapHex = "";

// Battery level
static int batteryLevel = -1; // -1 means unknown/not available

// Connection LED animation
static bool connectionLedEnabled = true;

// No-device pulsing LED state
static unsigned long noPairPulseStart = 0;
static bool noPairPulseActive = false;

// Async scan storage
struct DiscoveredDev { String addr; String name; int rssi; };
static std::vector<DiscoveredDev> discovered;
static volatile bool scanRunning = false;
static TaskHandle_t scanTaskHandle = nullptr;
static SemaphoreHandle_t discoveredMutex = nullptr;

// Async connect storage
static String pendingConnectAddr = "";
static String pendingConnectName = "";
static volatile bool connectRunning = false;
static TaskHandle_t connectTaskHandle = nullptr;
static volatile bool pairingComplete = false;

// Auto-reconnect storage
unsigned long lastAutoScanTime = 0;
const unsigned long AUTO_SCAN_INTERVAL = 5000; // Scan every 5 seconds
static volatile bool triggerAutoReconnect = false;
static bool firstBootAfterPairing = false;
static unsigned long bootTime = 0;
static int fastScanAttempts = 0;
const int MAX_FAST_SCANS = 5; // Only do 5 fast scans, then go back to normal

// Input delay to avoid accidental button presses
static uint8_t inputDelayMs = 50; // Default 50ms delay

// Battery polling
static unsigned long lastBatteryRead = 0;
const unsigned long BATTERY_READ_INTERVAL = 60000; // Read battery every 60 seconds
const unsigned long BATTERY_MIN_INTERVAL = 12000; // Minimum 12 seconds between reads (5 times per minute max)

// Forward declaration for LED animation functions
void playConnectionAnimation();
void updateNoPairPulse();
static void readBatteryLevel(NimBLEClient* c);

// -------------------- HTML UI --------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>HTPC-BLEBoot</title>
<style>
body{font-family:system-ui,Arial;margin:0;background:#111;color:#eee;padding-top:50px}
.topbar{position:fixed;top:0;left:0;right:0;background:#1d1d1d;border-bottom:2px solid #0a84ff;padding:10px 20px;display:flex;align-items:center;gap:15px;z-index:1000;box-shadow:0 2px 8px rgba(0,0,0,0.5)}
.topbar .logo{height:32px;display:flex;align-items:center;margin-right:auto;font-size:16px;font-weight:bold;color:#0a84ff}
.topbar .logo img{height:28px;width:auto;margin-right:8px}
.topbar .btn{background:#0a84ff;border:none;color:#fff;padding:8px 14px;border-radius:6px;cursor:pointer;text-decoration:none;font-size:13px}
.wrap{max-width:720px;margin:0 auto;padding:20px}
.card{background:#1d1d1d;border:1px solid #333;border-radius:8px;padding:16px;margin:16px 0}
.btn{background:#0a84ff;border:none;color:#fff;padding:10px 16px;border-radius:6px;cursor:pointer;text-decoration:none;display:inline-block}
.btn.small{padding:6px 10px;font-size:12px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
.dev{padding:8px;border-bottom:1px solid #333}
label{display:block;margin:8px 0}
code{white-space:pre-wrap;display:block;max-height:240px;overflow:auto;border:1px solid #333;padding:12px;border-radius:6px;background:#0d0d0d;font-family:monospace;font-size:12px;line-height:1.5}
.status-badge{display:inline-block;padding:4px 10px;border-radius:4px;font-size:12px;margin-left:10px}
.status-saved{background:#0a6;color:#fff}
.status-none{background:#666;color:#ddd}
</style></head>
<body>
<div class="topbar">
  <div class="logo">
    <svg width="28" height="28" viewBox="0 0 96 96" xmlns="http://www.w3.org/2000/svg"><rect fill="#0a84ff" x="8" y="8" width="80" height="80" rx="8"/><path fill="#fff" d="M24 32h16v8H24zm24 0h24v8H48zM24 48h48v8H24zm0 16h32v8H24z"/></svg>
    HTPC-BLEBoot
  </div>
  <a href="/" class="btn">Home</a>
  <a href="/mapping" class="btn">Mapping</a>
  <a href="/advanced" class="btn">Advanced</a>
</div>
<div class="wrap">

<div class="card">
  <div id="status">Status: idle</div>
  <div id="saved">Saved: none</div>
  <div id="battery" style="display:none;margin-top:4px;color:#888;font-size:13px"></div>
  <div class="row">
    <button class="btn" id="scan">Scan</button>
    <button class="btn" id="forget">Forget Device</button>
  </div>
  <label style="margin:8px 0;display:flex;align-items:center;gap:6px">
    <input type="checkbox" id="filterNamed" checked>
    <span style="font-size:13px">Only show devices with names</span>
  </label>
  <div id="scanlog" class="dev">Press Scan to search for BLE remotes...</div>
  <div id="list"></div>
</div>

<div class="card">
  <h3>Custom Button Mapping</h3>
  <p style="color:#888;margin:10px 0">Configure up to 64 custom button mappings with actions and LED colors.</p>
  <a href="/mapping" class="btn" style="display:inline-block;text-decoration:none">Open Mapping Page</a>
</div>

<div class="card">
  <h3>HID Descriptor Status</h3>
  <div id="descriptorStatus" style="margin:10px 0">
    <div>Saved Descriptor: <span id="descSaved" class="status-badge status-none">Not Saved</span></div>
    <div style="margin-top:8px;color:#888;font-size:13px">Current Length: <span id="descLen">0</span> bytes</div>
  </div>
  <div id="descriptorDisplay" style="margin-top:12px"></div>
</div>

<div class="card">
  <h3>About</h3>
  <p style="color:#888;margin:8px 0;font-size:14px">
    <strong>HTPC-BLEBoot</strong> - ESP32 BLE Remote Control to USB HID Bridge
  </p>
  <p style="color:#888;margin:12px 0;font-size:13px">
    This device connects to BLE remotes and allows custom button mappings with GPIO outputs, LED control, keyboard/media key forwarding, and hold-time actions.
  </p>
  <div style="margin-top:16px;padding-top:16px;border-top:1px solid #333">
    <div style="color:#888;font-size:12px">
      <strong>Network:</strong><br>
      AP SSID: <span id="aboutSsid">Loading...</span><br>
      mDNS: http://<span id="aboutMdns">Loading...</span>.local<br>
      IP Address: <span id="aboutIp">192.168.4.1</span>
    </div>
  </div>
  <div style="margin-top:16px;padding-top:16px;border-top:1px solid #333;text-align:center">
    <div style="font-weight:bold;color:#eee;margin-bottom:20px">Made by Liams Electronics Lab</div>
    <div style="margin-bottom:20px">
      <a href="https://www.youtube.com/channel/UCps0V_MhxlnIvX6RsPZBlxw" target="_blank" style="font-weight:bold;color:#cc0000;text-decoration:none;display:block;margin-bottom:10px">YOUTUBE</a>
      <div id="qrYoutube" style="display:inline-block;padding:8px;background:#fff"></div>
    </div>
    <div style="margin-top:24px;padding-top:24px;border-top:3px solid #444">
      <a href="https://github.com/Liams-Electronics-Lab" target="_blank" style="font-weight:bold;color:#6fa8dc;text-decoration:none;display:block;margin-bottom:10px">GITHUB</a>
      <div id="qrGithub" style="display:inline-block;padding:8px;background:#fff"></div>
    </div>
  </div>
</div>

<script>
// QRCode.js library - minified version
var QRCode;!function(){function a(a){this.mode=c.MODE_8BIT_BYTE,this.data=a,this.parsedData=[];for(var b=[],d=0,e=this.data.length;e>d;d++){var f=this.data.charCodeAt(d);f>65536?(b[0]=240|(1835008&f)>>>18,b[1]=128|(258048&f)>>>12,b[2]=128|(4032&f)>>>6,b[3]=128|63&f):f>2048?(b[0]=224|(61440&f)>>>12,b[1]=128|(4032&f)>>>6,b[2]=128|63&f):f>128?(b[0]=192|(1984&f)>>>6,b[1]=128|63&f):b[0]=f,this.parsedData=this.parsedData.concat(b)}this.parsedData.length!=this.data.length&&(this.parsedData.unshift(191),this.parsedData.unshift(187),this.parsedData.unshift(239))}function b(a,b){this.typeNumber=a,this.errorCorrectLevel=b,this.modules=null,this.moduleCount=0,this.dataCache=null,this.dataList=[]}function i(a,b){if(void 0==a.length)throw new Error(a.length+"/"+b);for(var c=0;c<a.length&&0==a[c];)c++;this.num=new Array(a.length-c+b);for(var d=0;d<a.length-c;d++)this.num[d]=a[d+c]}function j(a,b){this.totalCount=a,this.dataCount=b}function k(){this.buffer=[],this.length=0}function m(){return"undefined"!=typeof CanvasRenderingContext2D}function n(){var a=!1,b=navigator.userAgent;return/android/i.test(b)&&(a=!0,aMat=b.toString().match(/android ([0-9]\.[0-9])/i),aMat&&aMat[1]&&(a=parseFloat(aMat[1]))),a}function r(a,b){for(var c=1,e=s(a),f=0,g=l.length;g>=f;f++){var h=0;switch(b){case d.L:h=l[f][0];break;case d.M:h=l[f][1];break;case d.Q:h=l[f][2];break;case d.H:h=l[f][3]}if(h>=e)break;c++}if(c>l.length)throw new Error("Too long data");return c}function s(a){var b=encodeURI(a).toString().replace(/\%[0-9a-fA-F]{2}/g,"a");return b.length+(b.length!=a?3:0)}a.prototype={getLength:function(){return this.parsedData.length},write:function(a){for(var b=0,c=this.parsedData.length;c>b;b++)a.put(this.parsedData[b],8)}},b.prototype={addData:function(b){var c=new a(b);this.dataList.push(c),this.dataCache=null},isDark:function(a,b){if(0>a||this.moduleCount<=a||0>b||this.moduleCount<=b)throw new Error(a+","+b);return this.modules[a][b]},getModuleCount:function(){return this.moduleCount},make:function(){this.makeImpl(!1,this.getBestMaskPattern())},makeImpl:function(a,c){this.moduleCount=4*this.typeNumber+17,this.modules=new Array(this.moduleCount);for(var d=0;d<this.moduleCount;d++){this.modules[d]=new Array(this.moduleCount);for(var e=0;e<this.moduleCount;e++)this.modules[d][e]=null}this.setupPositionProbePattern(0,0),this.setupPositionProbePattern(this.moduleCount-7,0),this.setupPositionProbePattern(0,this.moduleCount-7),this.setupPositionAdjustPattern(),this.setupTimingPattern(),this.setupTypeInfo(a,c),this.typeNumber>=7&&this.setupTypeNumber(a),null==this.dataCache&&(this.dataCache=b.createData(this.typeNumber,this.errorCorrectLevel,this.dataList)),this.mapData(this.dataCache,c)},setupPositionProbePattern:function(a,b){for(var c=-1;7>=c;c++)if(!(-1>=a+c||this.moduleCount<=a+c))for(var d=-1;7>=d;d++)-1>=b+d||this.moduleCount<=b+d||(this.modules[a+c][b+d]=c>=0&&6>=c&&(0==d||6==d)||d>=0&&6>=d&&(0==c||6==c)||c>=2&&4>=c&&d>=2&&4>=d?!0:!1)},getBestMaskPattern:function(){for(var a=0,b=0,c=0;8>c;c++){this.makeImpl(!0,c);var d=f.getLostPoint(this);(0==c||a>d)&&(a=d,b=c)}return b},createMovieClip:function(a,b,c){var d=a.createEmptyMovieClip(b,c),e=1;this.make();for(var f=0;f<this.modules.length;f++)for(var g=f*e,h=0;h<this.modules[f].length;h++){var i=h*e,j=this.modules[f][h];j&&(d.beginFill(0,100),d.moveTo(i,g),d.lineTo(i+e,g),d.lineTo(i+e,g+e),d.lineTo(i,g+e),d.endFill())}return d},setupTimingPattern:function(){for(var a=8;a<this.moduleCount-8;a++)null==this.modules[a][6]&&(this.modules[a][6]=0==a%2);for(var b=8;b<this.moduleCount-8;b++)null==this.modules[6][b]&&(this.modules[6][b]=0==b%2)},setupPositionAdjustPattern:function(){for(var a=f.getPatternPosition(this.typeNumber),b=0;b<a.length;b++)for(var c=0;c<a.length;c++){var d=a[b],e=a[c];if(null==this.modules[d][e])for(var g=-2;2>=g;g++)for(var h=-2;2>=h;h++)this.modules[d+g][e+h]=-2==g||2==g||-2==h||2==h||0==g&&0==h?!0:!1}},setupTypeNumber:function(a){for(var b=f.getBCHTypeNumber(this.typeNumber),c=0;18>c;c++){var d=!a&&1==(1&b>>c);this.modules[Math.floor(c/3)][c%3+this.moduleCount-8-3]=d}for(var c=0;18>c;c++){var d=!a&&1==(1&b>>c);this.modules[c%3+this.moduleCount-8-3][Math.floor(c/3)]=d}},setupTypeInfo:function(a,b){for(var c=this.errorCorrectLevel<<3|b,d=f.getBCHTypeInfo(c),e=0;15>e;e++){var g=!a&&1==(1&d>>e);6>e?this.modules[e][8]=g:8>e?this.modules[e+1][8]=g:this.modules[this.moduleCount-15+e][8]=g}for(var e=0;15>e;e++){var g=!a&&1==(1&d>>e);8>e?this.modules[8][this.moduleCount-e-1]=g:9>e?this.modules[8][15-e-1+1]=g:this.modules[8][15-e-1]=g}this.modules[this.moduleCount-8][8]=!a},mapData:function(a,b){for(var c=-1,d=this.moduleCount-1,e=7,g=0,h=this.moduleCount-1;h>0;h-=2)for(6==h&&h--;;){for(var i=0;2>i;i++)if(null==this.modules[d][h-i]){var j=!1;g<a.length&&(j=1==(1&a[g]>>>e));var k=f.getMask(b,d,h-i);k&&(j=!j),this.modules[d][h-i]=j,e--,-1==e&&(g++,e=7)}if(d+=c,0>d||this.moduleCount<=d){d-=c,c=-c;break}}}},b.PAD0=236,b.PAD1=17,b.createData=function(a,c,d){for(var e=j.getRSBlocks(a,c),g=new k,h=0;h<d.length;h++){var i=d[h];g.put(i.mode,4),g.put(i.getLength(),f.getLengthInBits(i.mode,a)),i.write(g)}for(var l=0,h=0;h<e.length;h++)l+=e[h].dataCount;if(g.getLengthInBits()>8*l)throw new Error("code length overflow. ("+g.getLengthInBits()+">"+8*l+")");for(g.getLengthInBits()+4<=8*l&&g.put(0,4);0!=g.getLengthInBits()%8;)g.putBit(!1);for(;;){if(g.getLengthInBits()>=8*l)break;if(g.put(b.PAD0,8),g.getLengthInBits()>=8*l)break;g.put(b.PAD1,8)}return b.createBytes(g,e)},b.createBytes=function(a,b){for(var c=0,d=0,e=0,g=new Array(b.length),h=new Array(b.length),j=0;j<b.length;j++){var k=b[j].dataCount,l=b[j].totalCount-k;d=Math.max(d,k),e=Math.max(e,l),g[j]=new Array(k);for(var m=0;m<g[j].length;m++)g[j][m]=255&a.buffer[m+c];c+=k;var n=f.getErrorCorrectPolynomial(l),o=new i(g[j],n.getLength()-1),p=o.mod(n);h[j]=new Array(n.getLength()-1);for(var m=0;m<h[j].length;m++){var q=m+p.getLength()-h[j].length;h[j][m]=q>=0?p.get(q):0}}for(var r=0,m=0;m<b.length;m++)r+=b[m].totalCount;for(var s=new Array(r),t=0,m=0;d>m;m++)for(var j=0;j<b.length;j++)m<g[j].length&&(s[t++]=g[j][m]);for(var m=0;e>m;m++)for(var j=0;j<b.length;j++)m<h[j].length&&(s[t++]=h[j][m]);return s};for(var c={MODE_NUMBER:1,MODE_ALPHA_NUM:2,MODE_8BIT_BYTE:4,MODE_KANJI:8},d={L:1,M:0,Q:3,H:2},e={PATTERN000:0,PATTERN001:1,PATTERN010:2,PATTERN011:3,PATTERN100:4,PATTERN101:5,PATTERN110:6,PATTERN111:7},f={PATTERN_POSITION_TABLE:[[],[6,18],[6,22],[6,26],[6,30],[6,34],[6,22,38],[6,24,42],[6,26,46],[6,28,50],[6,30,54],[6,32,58],[6,34,62],[6,26,46,66],[6,26,48,70],[6,26,50,74],[6,30,54,78],[6,30,56,82],[6,30,58,86],[6,34,62,90],[6,28,50,72,94],[6,26,50,74,98],[6,30,54,78,102],[6,28,54,80,106],[6,32,58,84,110],[6,30,58,86,114],[6,34,62,90,118],[6,26,50,74,98,122],[6,30,54,78,102,126],[6,26,52,78,104,130],[6,30,56,82,108,134],[6,34,60,86,112,138],[6,30,58,86,114,142],[6,34,62,90,118,146],[6,30,54,78,102,126,150],[6,24,50,76,102,128,154],[6,28,54,80,106,132,158],[6,32,58,84,110,136,162],[6,26,54,82,110,138,166],[6,30,58,86,114,142,170]],G15:1335,G18:7973,G15_MASK:21522,getBCHTypeInfo:function(a){for(var b=a<<10;f.getBCHDigit(b)-f.getBCHDigit(f.G15)>=0;)b^=f.G15<<f.getBCHDigit(b)-f.getBCHDigit(f.G15);return(a<<10|b)^f.G15_MASK},getBCHTypeNumber:function(a){for(var b=a<<12;f.getBCHDigit(b)-f.getBCHDigit(f.G18)>=0;)b^=f.G18<<f.getBCHDigit(b)-f.getBCHDigit(f.G18);return a<<12|b},getBCHDigit:function(a){for(var b=0;0!=a;)b++,a>>>=1;return b},getPatternPosition:function(a){return f.PATTERN_POSITION_TABLE[a-1]},getMask:function(a,b,c){switch(a){case e.PATTERN000:return 0==(b+c)%2;case e.PATTERN001:return 0==b%2;case e.PATTERN010:return 0==c%3;case e.PATTERN011:return 0==(b+c)%3;case e.PATTERN100:return 0==(Math.floor(b/2)+Math.floor(c/3))%2;case e.PATTERN101:return 0==b*c%2+b*c%3;case e.PATTERN110:return 0==(b*c%2+b*c%3)%2;case e.PATTERN111:return 0==(b*c%3+(b+c)%2)%2;default:throw new Error("bad maskPattern:"+a)}},getErrorCorrectPolynomial:function(a){for(var b=new i([1],0),c=0;a>c;c++)b=b.multiply(new i([1,g.gexp(c)],0));return b},getLengthInBits:function(a,b){if(b>=1&&10>b)switch(a){case c.MODE_NUMBER:return 10;case c.MODE_ALPHA_NUM:return 9;case c.MODE_8BIT_BYTE:return 8;case c.MODE_KANJI:return 8;default:throw new Error("mode:"+a)}else if(27>b)switch(a){case c.MODE_NUMBER:return 12;case c.MODE_ALPHA_NUM:return 11;case c.MODE_8BIT_BYTE:return 16;case c.MODE_KANJI:return 10;default:throw new Error("mode:"+a)}else{if(!(41>b))throw new Error("type:"+b);switch(a){case c.MODE_NUMBER:return 14;case c.MODE_ALPHA_NUM:return 13;case c.MODE_8BIT_BYTE:return 16;case c.MODE_KANJI:return 12;default:throw new Error("mode:"+a)}}},getLostPoint:function(a){for(var b=a.getModuleCount(),c=0,d=0;b>d;d++)for(var e=0;b>e;e++){for(var f=0,g=a.isDark(d,e),h=-1;1>=h;h++)if(!(0>d+h||d+h>=b))for(var i=-1;1>=i;i++)0>e+i||e+i>=b||(0!=h||0!=i)&&g==a.isDark(d+h,e+i)&&f++;f>5&&(c+=3+f-5)}for(var d=0;b-1>d;d++)for(var e=0;b-1>e;e++){var j=0;a.isDark(d,e)&&j++,a.isDark(d+1,e)&&j++,a.isDark(d,e+1)&&j++,a.isDark(d+1,e+1)&&j++,(0==j||4==j)&&(c+=3)}for(var d=0;b>d;d++)for(var e=0;b-6>e;e++)a.isDark(d,e)&&!a.isDark(d,e+1)&&a.isDark(d,e+2)&&a.isDark(d,e+3)&&a.isDark(d,e+4)&&!a.isDark(d,e+5)&&a.isDark(d,e+6)&&(c+=40);for(var e=0;b>e;e++)for(var d=0;b-6>d;d++)a.isDark(d,e)&&!a.isDark(d+1,e)&&a.isDark(d+2,e)&&a.isDark(d+3,e)&&a.isDark(d+4,e)&&!a.isDark(d+5,e)&&a.isDark(d+6,e)&&(c+=40);for(var k=0,e=0;b>e;e++)for(var d=0;b>d;d++)a.isDark(d,e)&&k++;var l=Math.abs(100*k/b/b-50)/5;return c+=10*l}},g={glog:function(a){if(1>a)throw new Error("glog("+a+")");return g.LOG_TABLE[a]},gexp:function(a){for(;0>a;)a+=255;for(;a>=256;)a-=255;return g.EXP_TABLE[a]},EXP_TABLE:new Array(256),LOG_TABLE:new Array(256)},h=0;8>h;h++)g.EXP_TABLE[h]=1<<h;for(var h=8;256>h;h++)g.EXP_TABLE[h]=g.EXP_TABLE[h-4]^g.EXP_TABLE[h-5]^g.EXP_TABLE[h-6]^g.EXP_TABLE[h-8];for(var h=0;255>h;h++)g.LOG_TABLE[g.EXP_TABLE[h]]=h;i.prototype={get:function(a){return this.num[a]},getLength:function(){return this.num.length},multiply:function(a){for(var b=new Array(this.getLength()+a.getLength()-1),c=0;c<this.getLength();c++)for(var d=0;d<a.getLength();d++)b[c+d]^=g.gexp(g.glog(this.get(c))+g.glog(a.get(d)));return new i(b,0)},mod:function(a){if(this.getLength()-a.getLength()<0)return this;for(var b=g.glog(this.get(0))-g.glog(a.get(0)),c=new Array(this.getLength()),d=0;d<this.getLength();d++)c[d]=this.get(d);for(var d=0;d<a.getLength();d++)c[d]^=g.gexp(g.glog(a.get(d))+b);return new i(c,0).mod(a)}},j.RS_BLOCK_TABLE=[[1,26,19],[1,26,16],[1,26,13],[1,26,9],[1,44,34],[1,44,28],[1,44,22],[1,44,16],[1,70,55],[1,70,44],[2,35,17],[2,35,13],[1,100,80],[2,50,32],[2,50,24],[4,25,9],[1,134,108],[2,67,43],[2,33,15,2,34,16],[2,33,11,2,34,12],[2,86,68],[4,43,27],[4,43,19],[4,43,15],[2,98,78],[4,49,31],[2,32,14,4,33,15],[4,39,13,1,40,14],[2,121,97],[2,60,38,2,61,39],[4,40,18,2,41,19],[4,40,14,2,41,15],[2,146,116],[3,58,36,2,59,37],[4,36,16,4,37,17],[4,36,12,4,37,13],[2,86,68,2,87,69],[4,69,43,1,70,44],[6,43,19,2,44,20],[6,43,15,2,44,16],[4,101,81],[1,80,50,4,81,51],[4,50,22,4,51,23],[3,36,12,8,37,13],[2,116,92,2,117,93],[6,58,36,2,59,37],[4,46,20,6,47,21],[7,42,14,4,43,15],[4,133,107],[8,59,37,1,60,38],[8,44,20,4,45,21],[12,33,11,4,34,12],[3,145,115,1,146,116],[4,64,40,5,65,41],[11,36,16,5,37,17],[11,36,12,5,37,13],[5,109,87,1,110,88],[5,65,41,5,66,42],[5,54,24,7,55,25],[11,36,12],[5,122,98,1,123,99],[7,73,45,3,74,46],[15,43,19,2,44,20],[3,45,15,13,46,16],[1,135,107,5,136,108],[10,74,46,1,75,47],[1,50,22,15,51,23],[2,42,14,17,43,15],[5,150,120,1,151,121],[9,69,43,4,70,44],[17,50,22,1,51,23],[2,42,14,19,43,15],[3,141,113,4,142,114],[3,70,44,11,71,45],[17,47,21,4,48,22],[9,39,13,16,40,14],[3,135,107,5,136,108],[3,67,41,13,68,42],[15,54,24,5,55,25],[15,43,15,10,44,16],[4,144,116,4,145,117],[17,68,42],[17,50,22,6,51,23],[19,46,16,6,47,17],[2,139,111,7,140,112],[17,74,46],[7,54,24,16,55,25],[34,37,13],[4,151,121,5,152,122],[4,75,47,14,76,48],[11,54,24,14,55,25],[16,45,15,14,46,16],[6,147,117,4,148,118],[6,73,45,14,74,46],[11,54,24,16,55,25],[30,46,16,2,47,17],[8,132,106,4,133,107],[8,75,47,13,76,48],[7,54,24,22,55,25],[22,45,15,13,46,16],[10,142,114,2,143,115],[19,74,46,4,75,47],[28,50,22,6,51,23],[33,46,16,4,47,17],[8,152,122,4,153,123],[22,73,45,3,74,46],[8,53,23,26,54,24],[12,45,15,28,46,16],[3,147,117,10,148,118],[3,73,45,23,74,46],[4,54,24,31,55,25],[11,45,15,31,46,16],[7,146,116,7,147,117],[21,73,45,7,74,46],[1,53,23,37,54,24],[19,45,15,26,46,16],[5,145,115,10,146,116],[19,75,47,10,76,48],[15,54,24,25,55,25],[23,45,15,25,46,16],[13,145,115,3,146,116],[2,74,46,29,75,47],[42,54,24,1,55,25],[23,45,15,28,46,16],[17,145,115],[10,74,46,23,75,47],[10,54,24,35,55,25],[19,45,15,35,46,16],[17,145,115,1,146,116],[14,74,46,21,75,47],[29,54,24,19,55,25],[11,45,15,46,46,16],[13,145,115,6,146,116],[14,74,46,23,75,47],[44,54,24,7,55,25],[59,46,16,1,47,17],[12,151,121,7,152,122],[12,75,47,26,76,48],[39,54,24,14,55,25],[22,45,15,41,46,16],[6,151,121,14,152,122],[6,75,47,34,76,48],[46,54,24,10,55,25],[2,45,15,64,46,16],[17,152,122,4,153,123],[29,74,46,14,75,47],[49,54,24,10,55,25],[24,45,15,46,46,16],[4,152,122,18,153,123],[13,74,46,32,75,47],[48,54,24,14,55,25],[42,45,15,32,46,16],[20,147,117,4,148,118],[40,75,47,7,76,48],[43,54,24,22,55,25],[10,45,15,67,46,16],[19,148,118,6,149,119],[18,75,47,31,76,48],[34,54,24,34,55,25],[20,45,15,61,46,16]],j.getRSBlocks=function(a,b){var c=j.getRsBlockTable(a,b);if(void 0==c)throw new Error("bad rs block @ typeNumber:"+a+"/errorCorrectLevel:"+b);for(var d=c.length/3,e=[],f=0;d>f;f++)for(var g=c[3*f+0],h=c[3*f+1],i=c[3*f+2],k=0;g>k;k++)e.push(new j(h,i));return e},j.getRsBlockTable=function(a,b){switch(b){case d.L:return j.RS_BLOCK_TABLE[4*(a-1)+0];case d.M:return j.RS_BLOCK_TABLE[4*(a-1)+1];case d.Q:return j.RS_BLOCK_TABLE[4*(a-1)+2];case d.H:return j.RS_BLOCK_TABLE[4*(a-1)+3];default:return void 0}},k.prototype={get:function(a){var b=Math.floor(a/8);return 1==(1&this.buffer[b]>>>7-a%8)},put:function(a,b){for(var c=0;b>c;c++)this.putBit(1==(1&a>>>b-c-1))},getLengthInBits:function(){return this.length},putBit:function(a){var b=Math.floor(this.length/8);this.buffer.length<=b&&this.buffer.push(0),a&&(this.buffer[b]|=128>>>this.length%8),this.length++}};var l=[[17,14,11,7],[32,26,20,14],[53,42,32,24],[78,62,46,34],[106,84,60,44],[134,106,74,58],[154,122,86,64],[192,152,108,84],[230,180,130,98],[271,213,151,119],[321,251,177,137],[367,287,203,155],[425,331,241,177],[458,362,258,194],[520,412,292,220],[586,450,322,250],[644,504,364,280],[718,560,394,310],[792,624,442,338],[858,666,482,382],[929,711,509,403],[1003,779,565,439],[1091,857,611,461],[1171,911,661,511],[1273,997,715,535],[1367,1059,751,593],[1465,1125,805,625],[1528,1190,868,658],[1628,1264,908,698],[1732,1370,982,742],[1840,1452,1030,790],[1952,1538,1112,842],[2068,1628,1168,898],[2188,1722,1228,958],[2303,1809,1283,983],[2431,1911,1351,1051],[2563,1989,1423,1093],[2699,2099,1499,1139],[2809,2213,1579,1219],[2953,2331,1663,1273]],o=function(){var a=function(a,b){this._el=a,this._htOption=b};return a.prototype.draw=function(a){function g(a,b){var c=document.createElementNS("http://www.w3.org/2000/svg",a);for(var d in b)b.hasOwnProperty(d)&&c.setAttribute(d,b[d]);return c}var b=this._htOption,c=this._el,d=a.getModuleCount();Math.floor(b.width/d),Math.floor(b.height/d),this.clear();var h=g("svg",{viewBox:"0 0 "+String(d)+" "+String(d),width:"100%",height:"100%",fill:b.colorLight});h.setAttributeNS("http://www.w3.org/2000/xmlns/","xmlns:xlink","http://www.w3.org/1999/xlink"),c.appendChild(h),h.appendChild(g("rect",{fill:b.colorDark,width:"1",height:"1",id:"template"}));for(var i=0;d>i;i++)for(var j=0;d>j;j++)if(a.isDark(i,j)){var k=g("use",{x:String(i),y:String(j)});k.setAttributeNS("http://www.w3.org/1999/xlink","href","#template"),h.appendChild(k)}},a.prototype.clear=function(){for(;this._el.hasChildNodes();)this._el.removeChild(this._el.lastChild)},a}(),p="svg"===document.documentElement.tagName.toLowerCase(),q=p?o:m()?function(){function a(){this._elImage.src=this._elCanvas.toDataURL("image/png"),this._elImage.style.display="block",this._elCanvas.style.display="none"}function d(a,b){var c=this;if(c._fFail=b,c._fSuccess=a,null===c._bSupportDataURI){var d=document.createElement("img"),e=function(){c._bSupportDataURI=!1,c._fFail&&_fFail.call(c)},f=function(){c._bSupportDataURI=!0,c._fSuccess&&c._fSuccess.call(c)};return d.onabort=e,d.onerror=e,d.onload=f,d.src="data:image/gif;base64,iVBORw0KGgoAAAANSUhEUgAAAAUAAAAFCAYAAACNbyblAAAAHElEQVQI12P4//8/w38GIAXDIBKE0DHxgljNBAAO9TXL0Y4OHwAAAABJRU5ErkJggg==",void 0}c._bSupportDataURI===!0&&c._fSuccess?c._fSuccess.call(c):c._bSupportDataURI===!1&&c._fFail&&c._fFail.call(c)}if(this._android&&this._android<=2.1){var b=1/window.devicePixelRatio,c=CanvasRenderingContext2D.prototype.drawImage;CanvasRenderingContext2D.prototype.drawImage=function(a,d,e,f,g,h,i,j){if("nodeName"in a&&/img/i.test(a.nodeName))for(var l=arguments.length-1;l>=1;l--)arguments[l]=arguments[l]*b;else"undefined"==typeof j&&(arguments[1]*=b,arguments[2]*=b,arguments[3]*=b,arguments[4]*=b);c.apply(this,arguments)}}var e=function(a,b){this._bIsPainted=!1,this._android=n(),this._htOption=b,this._elCanvas=document.createElement("canvas"),this._elCanvas.width=b.width,this._elCanvas.height=b.height,a.appendChild(this._elCanvas),this._el=a,this._oContext=this._elCanvas.getContext("2d"),this._bIsPainted=!1,this._elImage=document.createElement("img"),this._elImage.style.display="none",this._el.appendChild(this._elImage),this._bSupportDataURI=null};return e.prototype.draw=function(a){var b=this._elImage,c=this._oContext,d=this._htOption,e=a.getModuleCount(),f=d.width/e,g=d.height/e,h=Math.round(f),i=Math.round(g);b.style.display="none",this.clear();for(var j=0;e>j;j++)for(var k=0;e>k;k++){var l=a.isDark(j,k),m=k*f,n=j*g;c.strokeStyle=l?d.colorDark:d.colorLight,c.lineWidth=1,c.fillStyle=l?d.colorDark:d.colorLight,c.fillRect(m,n,f,g),c.strokeRect(Math.floor(m)+.5,Math.floor(n)+.5,h,i),c.strokeRect(Math.ceil(m)-.5,Math.ceil(n)-.5,h,i)}this._bIsPainted=!0},e.prototype.makeImage=function(){this._bIsPainted&&d.call(this,a)},e.prototype.isPainted=function(){return this._bIsPainted},e.prototype.clear=function(){this._oContext.clearRect(0,0,this._elCanvas.width,this._elCanvas.height),this._bIsPainted=!1},e.prototype.round=function(a){return a?Math.floor(1e3*a)/1e3:a},e}():function(){var a=function(a,b){this._el=a,this._htOption=b};return a.prototype.draw=function(a){for(var b=this._htOption,c=this._el,d=a.getModuleCount(),e=Math.floor(b.width/d),f=Math.floor(b.height/d),g=['<table style="border:0;border-collapse:collapse;">'],h=0;d>h;h++){g.push("<tr>");for(var i=0;d>i;i++)g.push('<td style="border:0;border-collapse:collapse;padding:0;margin:0;width:'+e+"px;height:"+f+"px;background-color:"+(a.isDark(h,i)?b.colorDark:b.colorLight)+';"></td>');g.push("</tr>")}g.push("</table>"),c.innerHTML=g.join("");var j=c.childNodes[0],k=(b.width-j.offsetWidth)/2,l=(b.height-j.offsetHeight)/2;k>0&&l>0&&(j.style.margin=l+"px "+k+"px")},a.prototype.clear=function(){this._el.innerHTML=""},a}();QRCode=function(a,b){if(this._htOption={width:256,height:256,typeNumber:4,colorDark:"#000000",colorLight:"#ffffff",correctLevel:d.H},"string"==typeof b&&(b={text:b}),b)for(var c in b)this._htOption[c]=b[c];"string"==typeof a&&(a=document.getElementById(a)),this._android=n(),this._el=a,this._oQRCode=null,this._oDrawing=new q(this._el,this._htOption),this._htOption.text&&this.makeCode(this._htOption.text)},QRCode.prototype.makeCode=function(a){this._oQRCode=new b(r(a,this._htOption.correctLevel),this._htOption.correctLevel),this._oQRCode.addData(a),this._oQRCode.make(),this._el.title=a,this._oDrawing.draw(this._oQRCode),this.makeImage()},QRCode.prototype.makeImage=function(){"function"==typeof this._oDrawing.makeImage&&(!this._android||this._android>=3)&&this._oDrawing.makeImage()},QRCode.prototype.clear=function(){this._oDrawing.clear()},QRCode.CorrectLevel=d}();

async function refresh(){
  const r = await fetch('/status');
  const s = await r.json();
  let statusText = 'Status: ';
  if (s.connected) {
    statusText += 'connected to ' + (s.activeName||s.activeAddr);
  } else if (s.connecting) {
    statusText += 'connecting...';
  } else if (s.activeAddr && s.savedAddr) {
    statusText += 'disconnected - click Connect again to reconnect (bonding saved)';
  } else {
    statusText += 'idle';
  }
  document.getElementById('status').textContent = statusText;
  document.getElementById('saved').textContent = 'Saved: ' + (s.savedAddr?(s.savedName? (s.savedName+' ('+s.savedAddr+')'):s.savedAddr):'none');
  
  // Update battery level display
  const batteryDiv = document.getElementById('battery');
  if (s.connected && s.batteryLevel >= 0) {
    batteryDiv.textContent = 'Battery: ' + s.batteryLevel + '%';
    batteryDiv.style.display = 'block';
  } else {
    batteryDiv.style.display = 'none';
  }
  
  // Update About card network info
  if(s.apSsid) document.getElementById('aboutSsid').textContent = s.apSsid;
  if(s.mdnsHost) document.getElementById('aboutMdns').textContent = s.mdnsHost;
  
  // Update descriptor status
  const descr = await fetch('/descriptor/status');
  const descData = await descr.json();
  const savedBadge = document.getElementById('descSaved');
  if(descData.hasSaved){
    savedBadge.textContent = 'Saved';
    savedBadge.className = 'status-badge status-saved';
  } else {
    savedBadge.textContent = 'Not Saved';
    savedBadge.className = 'status-badge status-none';
  }
  document.getElementById('descLen').textContent = descData.currentLength || 0;
  
  // Format descriptor display
  if(s.hidMap && s.hidMap.length > 0){
    const hex = s.hidMap.replace(/\s+/g, '');
    let formatted = '';
    for(let i=0; i<hex.length; i+=32){
      formatted += hex.substr(i, 32) + '\n';
    }
    document.getElementById('descriptorDisplay').innerHTML = '<code>' + formatted.trim() + '</code>';
  } else {
    document.getElementById('descriptorDisplay').innerHTML = '<code>(No descriptor captured yet)</code>';
  }
}

let scanInProgress = false;
document.getElementById('scan').onclick = async ()=>{
  if (scanInProgress) {
    document.getElementById('scanlog').textContent = 'Restarting device...';
    return;
  }
  scanInProgress = true;
  document.getElementById('scan').disabled = true;
  document.getElementById('scanlog').textContent = 'Starting scan...';
  
  const scanResponse = await fetch('/scan');
  const scanData = await scanResponse.json();
  
  if (scanData.restarting) {
    document.getElementById('scanlog').textContent = 'Device restarting, page will reload...';
    setTimeout(() => location.reload(), 3000);
    return;
  }
  
  const box = document.getElementById('list');
  box.innerHTML = '';
  const t0 = Date.now();
  let timer = setInterval(async ()=>{
    const rs = await fetch('/scan_results');
    const data = await rs.json();
    const arr = data.devices || [];
    const filterNamed = document.getElementById('filterNamed').checked;
    const filtered = filterNamed ? arr.filter(d => d.name && !d.name.startsWith('[')) : arr;
    document.getElementById('scanlog').textContent = (data.running?'Scanning... ':'Scan done. ') + `Found ${arr.length} device(s)` + (filterNamed && filtered.length < arr.length ? ` (${filtered.length} shown)` : '');
    box.innerHTML = '';
    filtered.forEach(d=>{
      const div = document.createElement('div');
      div.className='dev';
      const name = d.name||'Unnamed';
      const addr = d.address;
      div.innerHTML = `<b>${name}</b> <small>${addr}</small> RSSI ${d.rssi} `;
      const btn = document.createElement('button'); btn.textContent='Connect'; btn.className='btn small';
      btn.onclick = async ()=>{ 
        const res = await fetch('/connect?addr='+encodeURIComponent(addr)+'&name='+encodeURIComponent(name)); 
        const json = await res.json();
        if (!json.ok && json.err) {
          alert('Connect failed: ' + json.err);
        }
        refresh(); 
      };
      div.appendChild(btn);
      box.appendChild(div);
    });
    if (!data.running || (Date.now()-t0)>7000) {
      clearInterval(timer);
      scanInProgress = false;
      document.getElementById('scan').disabled = false;
    }
  }, 600);
};

const disconnectBtn = document.getElementById('disconnect');
if (disconnectBtn) disconnectBtn.onclick = async()=>{ await fetch('/disconnect'); refresh(); };

document.getElementById('forget').onclick = async()=>{ 
  if (confirm('Forget saved device and clear bonding?')) {
    await fetch('/forget'); 
    refresh(); 
  }
};

// Toggle filter checkbox during scan
document.getElementById('filterNamed').onchange = ()=>{
  // Trigger a re-render of the current scan results
  const event = new Event('change');
  document.getElementById('filterNamed').dispatchEvent(event);
};

// Auto-reload detection on device restart
let consecutiveFailures = 0;
let wasRestarting = false;
setInterval(async () => {
  try {
    const response = await fetch('/status', { method: 'GET', cache: 'no-cache' });
    if (response.ok) {
      if (wasRestarting && consecutiveFailures > 2) {
        console.log('Device back online, reloading page...');
        location.reload();
      }
      consecutiveFailures = 0;
    } else {
      consecutiveFailures++;
    }
  } catch (e) {
    consecutiveFailures++;
    if (consecutiveFailures > 2) {
      wasRestarting = true;
    }
  }
}, 1000);

// Generate QR codes using QRCode.js library
new QRCode(document.getElementById('qrYoutube'), {
  text: 'https://www.youtube.com/channel/UCps0V_MhxlnIvX6RsPZBlxw',
  width: 150,
  height: 150,
  colorDark: '#000000',
  colorLight: '#ffffff',
  correctLevel: QRCode.CorrectLevel.M
});

new QRCode(document.getElementById('qrGithub'), {
  text: 'https://github.com/Liams-Electronics-Lab',
  width: 150,
  height: 150,
  colorDark: '#000000',
  colorLight: '#ffffff',
  correctLevel: QRCode.CorrectLevel.M
});

refresh(); setInterval(refresh, 4000);
</script>
</div></body></html>
)HTML";

// -------------------- Mapping Page HTML --------------------
static const char MAPPING_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>Button Mapping - HTPC-BLEBoot</title>
<style>
body{font-family:system-ui,Arial;margin:0;background:#111;color:#eee;padding-top:50px}
.topbar{position:fixed;top:0;left:0;right:0;background:#1d1d1d;border-bottom:2px solid #0a84ff;padding:10px 20px;display:flex;align-items:center;gap:15px;z-index:1000;box-shadow:0 2px 8px rgba(0,0,0,0.5)}
.topbar .logo{height:32px;display:flex;align-items:center;margin-right:auto;font-size:16px;font-weight:bold;color:#0a84ff}
.topbar .logo svg{margin-right:8px}
.topbar .btn{background:#0a84ff;border:none;color:#fff;padding:8px 14px;border-radius:6px;cursor:pointer;text-decoration:none;font-size:13px}
.wrap{max-width:720px;margin:0 auto;padding:20px}
.card{background:#1d1d1d;border:1px solid #333;border-radius:8px;padding:16px;margin:16px 0}
.btn{background:#0a84ff;border:none;color:#fff;padding:10px 16px;border-radius:6px;cursor:pointer;text-decoration:none;display:inline-block}
.btn.small{padding:6px 10px;font-size:12px}
.btn.danger{background:#d00}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
label{display:block;margin:8px 0}
input[type="checkbox"]{transform:scale(1.1)}
</style></head>
<body>
<div class="topbar">
  <div class="logo">
    <svg width="28" height="28" viewBox="0 0 96 96" xmlns="http://www.w3.org/2000/svg"><rect fill="#0a84ff" x="8" y="8" width="80" height="80" rx="8"/><path fill="#fff" d="M24 32h16v8H24zm24 0h24v8H48zM24 48h48v8H24zm0 16h32v8H24z"/></svg>
    HTPC-BLEBoot
  </div>
  <a href="/" class="btn">Home</a>
  <a href="/mapping" class="btn">Mapping</a>
  <a href="/advanced" class="btn">Advanced</a>
</div>
<div class="wrap"

<div class="card">
  <button class="btn" id="addMapBtn">+ Add New Button Map</button>
  <p style="color:#888;margin:10px 0;font-size:14px">Create up to 64 custom button mappings. Click Learn to capture a button, then choose an action.</p>
</div>

<div id="buttonMaps"></div>

<script>
let learnTimers = {};

async function loadMaps(){
  const r = await fetch('/status');
  const s = await r.json();
  renderButtonMaps(s.maps || []);
}

function renderButtonMaps(maps) {
  const container = document.getElementById('buttonMaps');
  container.innerHTML = '';
  
  if (maps.length === 0) {
    container.innerHTML = '<div class="card" style="text-align:center;color:#888;padding:40px">No button maps yet. Click "+ Add New Button Map" to get started.</div>';
    return;
  }
  
  maps.forEach((map, idx) => {
    const card = document.createElement('div');
    card.className = 'card';
    card.style.marginBottom = '12px';
    card.style.background = '#252525';
    
    // Convert RGB to hex for color picker
    const hexColor = '#' + 
      (map.ledR || 0).toString(16).padStart(2, '0') + 
      (map.ledG || 0).toString(16).padStart(2, '0') + 
      (map.ledB || 0).toString(16).padStart(2, '0');
    
    card.innerHTML = `
      <div style="margin-bottom:12px">
        <input type="text" id="label${map.idx}" value="${map.label}" 
               placeholder="Button Label" 
               style="width:100%;padding:8px;font-size:14px;background:#1d1d1d;color:#eee;border:1px solid #444;border-radius:4px"
               onchange="updateMapLabel(${map.idx}, this.value)">
      </div>
      <div style="margin-bottom:8px">
        <button class="btn" onclick="learnButton(${map.idx})">Learn Button (10s)</button>
        <div id="learnStatus${map.idx}" style="margin-top:8px;color:#888;font-size:12px"></div>
      </div>
      <label style="font-weight:bold;margin-top:8px">${map.label} Action:</label>
      <select id="action${map.idx}" onchange="updateMapAction(${map.idx})" style="width:100%;margin:5px 0;padding:6px;background:#1d1d1d;color:#eee;border:1px solid #444">
        <option value="1" ${map.action==1?'selected':''}>GPIO</option>
        <option value="2" ${map.action==2?'selected':''}>LED</option>
        <option value="4" ${map.action==4?'selected':''}>Keyboard</option>
        <option value="5" ${map.action==5?'selected':''}>Media</option>
      </select>
      <div id="gpio${map.idx}" style="${map.action==1?'':'display:none'}">
        <label>GPIO Pin: <input type="number" id="gpioPin${map.idx}" value="${map.gpioPin}" min="0" max="48" 
               style="width:80px;padding:4px;background:#1d1d1d;color:#eee;border:1px solid #444" 
               onchange="updateMapAction(${map.idx})"></label>
        <label style="margin-top:8px">GPIO Mode:
          <select id="gpioMode${map.idx}" onchange="updateMapAction(${map.idx})" 
                  style="width:100%;padding:6px;background:#1d1d1d;color:#eee;border:1px solid #444;border-radius:4px">
            <option value="0" ${map.gpioMode==0?'selected':''}>Toggle (flip-flop)</option>
            <option value="1" ${map.gpioMode==1?'selected':''}>Pull HIGH</option>
            <option value="2" ${map.gpioMode==2?'selected':''}>Pull LOW</option>
          </select>
        </label>
        <label style="margin-top:8px">Time to Live (ms):
          <input type="number" id="ttl${map.idx}" value="${map.ttlMs || 0}" min="0" max="9999" 
                 style="width:100%;padding:4px;background:#1d1d1d;color:#eee;border:1px solid #444" 
                 onchange="updateMapAction(${map.idx})" placeholder="0 = hold state forever">
        </label>
        <p style="color:#888;margin:4px 0;font-size:11px">0 = hold forever, 1-9999 = auto-revert after ms</p>
        <label style="margin-top:8px">After TTL Expires:
          <select id="gpioTtlBehavior${map.idx}" onchange="updateMapAction(${map.idx})" 
                  style="width:100%;padding:6px;background:#1d1d1d;color:#eee;border:1px solid #444;border-radius:4px">
            <option value="0" ${map.gpioTtlBehavior==0?'selected':''}>Switch to opposite state</option>
            <option value="1" ${map.gpioTtlBehavior==1?'selected':''}>Float pin (INPUT)</option>
          </select>
        </label>
        <p style="color:#888;margin:4px 0;font-size:11px">Opposite = HIGH→LOW or LOW→HIGH, Float = high impedance</p>
      </div>
      <div id="ledColor${map.idx}" style="${map.action==2?'':'display:none'};margin:8px 0">
        <label style="display:flex;align-items:center;gap:8px">
          LED Color: 
          <input type="color" id="colorPicker${map.idx}" value="${hexColor}" 
                 style="width:60px;height:32px;border:1px solid #444;background:#1d1d1d;cursor:pointer" 
                 onchange="updateMapColor(${map.idx}, this.value)">
          <span id="colorPreview${map.idx}" style="padding:4px 12px;border-radius:4px;background:${hexColor};color:#fff;font-size:11px;font-family:monospace">${hexColor.toUpperCase()}</span>
        </label>
        <label style="margin-top:8px">Time to Live (ms):
          <input type="number" id="ttl${map.idx}" value="${map.ttlMs || 0}" min="0" max="9999" 
                 style="width:100%;padding:4px;background:#1d1d1d;color:#eee;border:1px solid #444" 
                 onchange="updateMapAction(${map.idx})" placeholder="0 = stay on while pressed">
        </label>
        <p style="color:#888;margin:4px 0;font-size:11px">0 = off on release, 1-9999 = auto-off after ms</p>
      </div>
      <div id="keyboard${map.idx}" style="${map.action==4?'':'display:none'};margin:8px 0">
        <label style="margin-bottom:8px">
          Key Code:
          <select id="keyCode${map.idx}" style="width:100%;padding:6px;background:#1d1d1d;color:#eee;border:1px solid #444;border-radius:4px;margin-top:4px" onchange="updateMapAction(${map.idx})">
            <optgroup label="Letters">
              <option value="0x04" ${map.keyCode==0x04?'selected':''}>A</option>
              <option value="0x05" ${map.keyCode==0x05?'selected':''}>B</option>
              <option value="0x06" ${map.keyCode==0x06?'selected':''}>C</option>
              <option value="0x07" ${map.keyCode==0x07?'selected':''}>D</option>
              <option value="0x08" ${map.keyCode==0x08?'selected':''}>E</option>
              <option value="0x09" ${map.keyCode==0x09?'selected':''}>F</option>
              <option value="0x0A" ${map.keyCode==0x0A?'selected':''}>G</option>
              <option value="0x0B" ${map.keyCode==0x0B?'selected':''}>H</option>
              <option value="0x0C" ${map.keyCode==0x0C?'selected':''}>I</option>
              <option value="0x0D" ${map.keyCode==0x0D?'selected':''}>J</option>
              <option value="0x0E" ${map.keyCode==0x0E?'selected':''}>K</option>
              <option value="0x0F" ${map.keyCode==0x0F?'selected':''}>L</option>
              <option value="0x10" ${map.keyCode==0x10?'selected':''}>M</option>
              <option value="0x11" ${map.keyCode==0x11?'selected':''}>N</option>
              <option value="0x12" ${map.keyCode==0x12?'selected':''}>O</option>
              <option value="0x13" ${map.keyCode==0x13?'selected':''}>P</option>
              <option value="0x14" ${map.keyCode==0x14?'selected':''}>Q</option>
              <option value="0x15" ${map.keyCode==0x15?'selected':''}>R</option>
              <option value="0x16" ${map.keyCode==0x16?'selected':''}>S</option>
              <option value="0x17" ${map.keyCode==0x17?'selected':''}>T</option>
              <option value="0x18" ${map.keyCode==0x18?'selected':''}>U</option>
              <option value="0x19" ${map.keyCode==0x19?'selected':''}>V</option>
              <option value="0x1A" ${map.keyCode==0x1A?'selected':''}>W</option>
              <option value="0x1B" ${map.keyCode==0x1B?'selected':''}>X</option>
              <option value="0x1C" ${map.keyCode==0x1C?'selected':''}>Y</option>
              <option value="0x1D" ${map.keyCode==0x1D?'selected':''}>Z</option>
            </optgroup>
            <optgroup label="Numbers">
              <option value="0x1E" ${map.keyCode==0x1E?'selected':''}>1</option>
              <option value="0x1F" ${map.keyCode==0x1F?'selected':''}>2</option>
              <option value="0x20" ${map.keyCode==0x20?'selected':''}>3</option>
              <option value="0x21" ${map.keyCode==0x21?'selected':''}>4</option>
              <option value="0x22" ${map.keyCode==0x22?'selected':''}>5</option>
              <option value="0x23" ${map.keyCode==0x23?'selected':''}>6</option>
              <option value="0x24" ${map.keyCode==0x24?'selected':''}>7</option>
              <option value="0x25" ${map.keyCode==0x25?'selected':''}>8</option>
              <option value="0x26" ${map.keyCode==0x26?'selected':''}>9</option>
              <option value="0x27" ${map.keyCode==0x27?'selected':''}>0</option>
            </optgroup>
            <optgroup label="Special Keys">
              <option value="0x28" ${map.keyCode==0x28?'selected':''}>Enter</option>
              <option value="0x29" ${map.keyCode==0x29?'selected':''}>Escape</option>
              <option value="0x2A" ${map.keyCode==0x2A?'selected':''}>Backspace</option>
              <option value="0x2B" ${map.keyCode==0x2B?'selected':''}>Tab</option>
              <option value="0x2C" ${map.keyCode==0x2C?'selected':''}>Spacebar</option>
            </optgroup>
            <optgroup label="Navigation">
              <option value="0x4F" ${map.keyCode==0x4F?'selected':''}>Right Arrow</option>
              <option value="0x50" ${map.keyCode==0x50?'selected':''}>Left Arrow</option>
              <option value="0x51" ${map.keyCode==0x51?'selected':''}>Down Arrow</option>
              <option value="0x52" ${map.keyCode==0x52?'selected':''}>Up Arrow</option>
              <option value="0x4A" ${map.keyCode==0x4A?'selected':''}>Home</option>
              <option value="0x4D" ${map.keyCode==0x4D?'selected':''}>End</option>
              <option value="0x4B" ${map.keyCode==0x4B?'selected':''}>Page Up</option>
              <option value="0x4E" ${map.keyCode==0x4E?'selected':''}>Page Down</option>
              <option value="0x49" ${map.keyCode==0x49?'selected':''}>Insert</option>
              <option value="0x4C" ${map.keyCode==0x4C?'selected':''}>Delete</option>
            </optgroup>
            <optgroup label="Function Keys">
              <option value="0x3A" ${map.keyCode==0x3A?'selected':''}>F1</option>
              <option value="0x3B" ${map.keyCode==0x3B?'selected':''}>F2</option>
              <option value="0x3C" ${map.keyCode==0x3C?'selected':''}>F3</option>
              <option value="0x3D" ${map.keyCode==0x3D?'selected':''}>F4</option>
              <option value="0x3E" ${map.keyCode==0x3E?'selected':''}>F5</option>
              <option value="0x3F" ${map.keyCode==0x3F?'selected':''}>F6</option>
              <option value="0x40" ${map.keyCode==0x40?'selected':''}>F7</option>
              <option value="0x41" ${map.keyCode==0x41?'selected':''}>F8</option>
              <option value="0x42" ${map.keyCode==0x42?'selected':''}>F9</option>
              <option value="0x43" ${map.keyCode==0x43?'selected':''}>F10</option>
              <option value="0x44" ${map.keyCode==0x44?'selected':''}>F11</option>
              <option value="0x45" ${map.keyCode==0x45?'selected':''}>F12</option>
            </optgroup>
          </select>
        </label>
        <label style="margin-top:8px">
          Modifier Keys:
          <div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:4px">
            <label style="display:flex;align-items:center;gap:4px">
              <input type="checkbox" id="modCtrl${map.idx}" ${(map.keyModifier&0x01)?'checked':''} onchange="updateMapAction(${map.idx})">
              <span>Ctrl</span>
            </label>
            <label style="display:flex;align-items:center;gap:4px">
              <input type="checkbox" id="modShift${map.idx}" ${(map.keyModifier&0x02)?'checked':''} onchange="updateMapAction(${map.idx})">
              <span>Shift</span>
            </label>
            <label style="display:flex;align-items:center;gap:4px">
              <input type="checkbox" id="modAlt${map.idx}" ${(map.keyModifier&0x04)?'checked':''} onchange="updateMapAction(${map.idx})">
              <span>Alt</span>
            </label>
            <label style="display:flex;align-items:center;gap:4px">
              <input type="checkbox" id="modWin${map.idx}" ${(map.keyModifier&0x08)?'checked':''} onchange="updateMapAction(${map.idx})">
              <span>Win</span>
            </label>
          </div>
        </label>
        <p style="color:#888;margin:4px 0;font-size:11px">Example: Remap media button to Backspace key</p>
      </div>
      <div id="media${map.idx}" style="${map.action==5?'':'display:none'};margin:8px 0">
        <label style="margin-bottom:8px">
          Media Control:
          <select id="mediaCode${map.idx}" style="width:100%;padding:6px;background:#1d1d1d;color:#eee;border:1px solid #444;border-radius:4px;margin-top:4px" onchange="updateMapAction(${map.idx})">
            <option value="0x0030" ${map.mediaCode==0x0030?'selected':''}>Power</option>
            <option value="0x0032" ${map.mediaCode==0x0032?'selected':''}>Sleep</option>
            <option value="0x0033" ${map.mediaCode==0x0033?'selected':''}>Wake Up</option>
            <option value="0x00B0" ${map.mediaCode==0x00B0?'selected':''}>Play</option>
            <option value="0x00B1" ${map.mediaCode==0x00B1?'selected':''}>Pause</option>
            <option value="0x00B2" ${map.mediaCode==0x00B2?'selected':''}>Record</option>
            <option value="0x00B3" ${map.mediaCode==0x00B3?'selected':''}>Fast Forward</option>
            <option value="0x00B4" ${map.mediaCode==0x00B4?'selected':''}>Rewind</option>
            <option value="0x00B5" ${map.mediaCode==0x00B5?'selected':''}>Next Track</option>
            <option value="0x00B6" ${map.mediaCode==0x00B6?'selected':''}>Previous Track</option>
            <option value="0x00B7" ${map.mediaCode==0x00B7?'selected':''}>Stop</option>
            <option value="0x00B8" ${map.mediaCode==0x00B8?'selected':''}>Eject</option>
            <option value="0x00B9" ${map.mediaCode==0x00B9?'selected':''}>Random Play</option>
            <option value="0x00BC" ${map.mediaCode==0x00BC?'selected':''}>Repeat</option>
            <option value="0x00CD" ${map.mediaCode==0x00CD?'selected':''}>Play/Pause Toggle</option>
            <option value="0x00E2" ${map.mediaCode==0x00E2?'selected':''}>Mute</option>
            <option value="0x00E9" ${map.mediaCode==0x00E9?'selected':''}>Volume Up</option>
            <option value="0x00EA" ${map.mediaCode==0x00EA?'selected':''}>Volume Down</option>
            <option value="0x0183" ${map.mediaCode==0x0183?'selected':''}>Media Select</option>
            <option value="0x018A" ${map.mediaCode==0x018A?'selected':''}>Mail</option>
            <option value="0x0192" ${map.mediaCode==0x0192?'selected':''}>Calculator</option>
            <option value="0x0221" ${map.mediaCode==0x0221?'selected':''}>WWW Search</option>
            <option value="0x0223" ${map.mediaCode==0x0223?'selected':''}>WWW Home</option>
            <option value="0x0224" ${map.mediaCode==0x0224?'selected':''}>WWW Back</option>
            <option value="0x0225" ${map.mediaCode==0x0225?'selected':''}>WWW Forward</option>
            <option value="0x0226" ${map.mediaCode==0x0226?'selected':''}>WWW Stop</option>
            <option value="0x0227" ${map.mediaCode==0x0227?'selected':''}>WWW Refresh</option>
            <option value="0x022A" ${map.mediaCode==0x022A?'selected':''}>WWW Favorites</option>
          </select>
        </label>
        <p style="color:#888;margin:4px 0;font-size:11px">Send consumer control commands (Volume, Play/Pause, etc.)</p>
      </div>
      <div id="passToUsbDiv${map.idx}" style="${(map.action==1||map.action==2)?'':'display:none'}">
        <label><input type="checkbox" id="pass${map.idx}" ${map.passToUsb?'checked':''} onchange="updateMapAction(${map.idx})"> Also forward to USB</label>
      </div>
      <hr style="border:none;border-top:1px solid #333;margin:12px 0">
      <div style="background:#1a1a1a;padding:10px;border-radius:4px;border:1px solid #333">
        <label style="display:block;margin-bottom:8px">
          <input type="checkbox" id="useHold${map.idx}" ${map.useHoldTime?'checked':''} onchange="updateHoldTimeVisibility(${map.idx})"> 
          <strong>Use Hold Time</strong>
        </label>
        <div id="holdSettings${map.idx}" style="${map.useHoldTime?'':'display:none'}">
          <label style="display:block;margin-bottom:8px">
            Hold Duration (seconds):
            <input type="number" id="holdSec${map.idx}" value="${map.holdTimeSec || 3}" min="1" max="10" 
                   style="width:100%;padding:4px;background:#1d1d1d;color:#eee;border:1px solid #444;margin-top:4px">
          </label>
          <label style="display:block;margin-bottom:8px">
            <input type="checkbox" id="applyUsb${map.idx}" ${map.applyToUsb?'checked':''}>
            Also apply to USB data
          </label>
          <p style="color:#888;margin:4px 0;font-size:11px">
            When enabled: Action executes after button is held for specified time. 
            Early release cancels the action.
            <br><br>
            <strong>USB behavior:</strong><br>
            - Checked: Single keystroke sent when hold time met<br>
            - Unchecked: Keystroke repeats continuously while held
          </p>
        </div>
      </div>
      <div style="margin-top:12px;display:flex;gap:8px">
        <button class="btn" onclick="saveMap(${map.idx})" style="flex:1">Save Changes</button>
        <button class="btn danger" onclick="deleteMap(${map.idx})" style="flex:1">Delete</button>
      </div>
    `;
    
    container.appendChild(card);
  });
}

async function updateMapLabel(idx, label) {
  await fetch(`/map/update?idx=${idx}&label=${encodeURIComponent(label)}`);
  // Don't auto-refresh to avoid closing color picker
}

async function updateMapAction(idx) {
  const action = document.getElementById(`action${idx}`).value;
  const gpioPin = document.getElementById(`gpioPin${idx}`)?.value || 2;
  const gpioMode = document.getElementById(`gpioMode${idx}`)?.value || 0;
  const gpioTtlBehavior = document.getElementById(`gpioTtlBehavior${idx}`)?.value || 0;
  const passToUsb = document.getElementById(`pass${idx}`).checked;
  
  // Show/hide GPIO field
  const gpioDiv = document.getElementById(`gpio${idx}`);
  if (gpioDiv) gpioDiv.style.display = action == 1 ? '' : 'none';
  
  // Show/hide LED color picker
  const ledColorDiv = document.getElementById(`ledColor${idx}`);
  if (ledColorDiv) ledColorDiv.style.display = action == 2 ? '' : 'none';
  
  // Show/hide keyboard remap settings
  const keyboardDiv = document.getElementById(`keyboard${idx}`);
  if (keyboardDiv) keyboardDiv.style.display = action == 4 ? '' : 'none';
  
  // Show/hide media control settings
  const mediaDiv = document.getElementById(`media${idx}`);
  if (mediaDiv) mediaDiv.style.display = action == 5 ? '' : 'none';
  
  // Show/hide "Also forward to USB" checkbox (only for GPIO and LED actions)
  const passToUsbDiv = document.getElementById(`passToUsbDiv${idx}`);
  if (passToUsbDiv) passToUsbDiv.style.display = (action == 1 || action == 2) ? '' : 'none';
  
  // Get keyboard settings if action is keyboard remap
  let keyCode = '0x2A'; // Default: Backspace
  let keyMod = 0;
  if (action == 4) {
    const keyCodeSelect = document.getElementById(`keyCode${idx}`);
    if (keyCodeSelect) keyCode = keyCodeSelect.value;
    
    // Calculate modifier byte
    if (document.getElementById(`modCtrl${idx}`)?.checked) keyMod |= 0x01;
    if (document.getElementById(`modShift${idx}`)?.checked) keyMod |= 0x02;
    if (document.getElementById(`modAlt${idx}`)?.checked) keyMod |= 0x04;
    if (document.getElementById(`modWin${idx}`)?.checked) keyMod |= 0x08;
  }
  
  // Get media control code if action is media
  let mediaCode = '0x00E9'; // Default: Volume Up
  if (action == 5) {
    const mediaCodeSelect = document.getElementById(`mediaCode${idx}`);
    if (mediaCodeSelect) mediaCode = mediaCodeSelect.value;
  }
  
  // Get TTL from the currently visible section
  let ttl = 0;
  if (action == 1 && gpioDiv) {
    const ttlInputs = gpioDiv.querySelectorAll('input[type="number"]');
    for (let input of ttlInputs) {
      if (input.placeholder && input.placeholder.includes('hold')) {
        ttl = parseInt(input.value) || 0;
        break;
      }
    }
  } else if (action == 2 && ledColorDiv) {
    const ttlInputs = ledColorDiv.querySelectorAll('input[type="number"]');
    for (let input of ttlInputs) {
      if (input.placeholder && input.placeholder.includes('pressed')) {
        ttl = parseInt(input.value) || 0;
        break;
      }
    }
  }
  
  await fetch(`/map/update?idx=${idx}&action=${action}&gpio=${gpioPin}&gpioMode=${gpioMode}&gpioTtlBehavior=${gpioTtlBehavior}&ttl=${ttl}&pass=${passToUsb?'1':'0'}&keyCode=${encodeURIComponent(keyCode)}&keyMod=${keyMod}&mediaCode=${encodeURIComponent(mediaCode)}`);
}

async function updateMapColor(idx, hexColor) {
  // Update the color preview text
  const preview = document.getElementById(`colorPreview${idx}`);
  if (preview) {
    preview.textContent = hexColor.toUpperCase();
    preview.style.background = hexColor;
  }
  
  await fetch(`/map/update?idx=${idx}&color=${encodeURIComponent(hexColor)}`);
}

function updateHoldTimeVisibility(idx) {
  const useHold = document.getElementById(`useHold${idx}`).checked;
  const holdSettings = document.getElementById(`holdSettings${idx}`);
  if (holdSettings) {
    holdSettings.style.display = useHold ? '' : 'none';
  }
}

async function saveMap(idx) {
  const label = document.getElementById(`label${idx}`)?.value || '';
  const action = document.getElementById(`action${idx}`).value;
  const gpioPin = document.getElementById(`gpioPin${idx}`)?.value || 2;
  const gpioMode = document.getElementById(`gpioMode${idx}`)?.value || 0;
  const gpioTtlBehavior = document.getElementById(`gpioTtlBehavior${idx}`)?.value || 0;
  const passToUsb = document.getElementById(`pass${idx}`).checked;
  const useHold = document.getElementById(`useHold${idx}`)?.checked || false;
  const holdSec = document.getElementById(`holdSec${idx}`)?.value || 3;
  const applyUsb = document.getElementById(`applyUsb${idx}`)?.checked || false;
  
  // Get TTL from the currently visible section
  let ttl = 0;
  const gpioDiv = document.getElementById(`gpio${idx}`);
  const ledDiv = document.getElementById(`ledColor${idx}`);
  
  if (action == 1 && gpioDiv) {
    const ttlInputs = gpioDiv.querySelectorAll('input[type="number"]');
    for (let input of ttlInputs) {
      if (input.placeholder && input.placeholder.includes('hold')) {
        ttl = parseInt(input.value) || 0;
        break;
      }
    }
  } else if (action == 2 && ledDiv) {
    const ttlInputs = ledDiv.querySelectorAll('input[type="number"]');
    for (let input of ttlInputs) {
      if (input.placeholder && input.placeholder.includes('pressed')) {
        ttl = parseInt(input.value) || 0;
        break;
      }
    }
  }
  
  console.log(`Saving map ${idx}: action=${action}, ttl=${ttl}, useHold=${useHold}, holdSec=${holdSec}, applyUsb=${applyUsb}`);
  
  // Get color if LED action
  let colorParam = '';
  if (action == 2) {
    const colorPicker = document.getElementById(`colorPicker${idx}`);
    if (colorPicker) {
      colorParam = `&color=${encodeURIComponent(colorPicker.value)}`;
    }
  }
  
  await fetch(`/map/update?idx=${idx}&label=${encodeURIComponent(label)}&action=${action}&gpio=${gpioPin}&gpioMode=${gpioMode}&gpioTtlBehavior=${gpioTtlBehavior}&ttl=${ttl}&useHold=${useHold?'1':'0'}&holdSec=${holdSec}&applyUsb=${applyUsb?'1':'0'}&pass=${passToUsb?'1':'0'}${colorParam}`);
  alert('Settings saved!');
}

async function deleteMap(idx) {
  if (confirm('Delete this button map?')) {
    await fetch(`/map/delete?idx=${idx}`);
    loadMaps(); // Refresh after delete
  }
}

async function learnButton(idx) {
  const statusDiv = document.getElementById(`learnStatus${idx}`);
  statusDiv.textContent = 'Learning... Press the button';
  statusDiv.style.color = '#0a84ff';
  
  if (learnTimers[idx]) clearInterval(learnTimers[idx]);
  
  await fetch(`/learn?idx=${idx}`);
  let countdown = 10;
  learnTimers[idx] = setInterval(async ()=>{
    const r = await fetch('/status');
    const s = await r.json();
    if (!s.learning) {
      clearInterval(learnTimers[idx]);
      delete learnTimers[idx];
      statusDiv.textContent = 'Button learned!';
      statusDiv.style.color = '#0f0';
      setTimeout(()=>{ statusDiv.textContent = ''; }, 3000);
      return;
    }
    countdown--;
    if (countdown <= 0) {
      clearInterval(learnTimers[idx]);
      delete learnTimers[idx];
      statusDiv.textContent = 'Learn timeout';
      statusDiv.style.color = '#f00';
      setTimeout(()=>{ statusDiv.textContent = ''; }, 3000);
    } else {
      statusDiv.textContent = `Learning... (${countdown}s)`;
    }
  }, 1000);
}

document.getElementById('addMapBtn').onclick = async()=>{
  await fetch('/map/add');
  loadMaps(); // Refresh after adding
};

loadMaps(); // Initial load, NO auto-refresh timer

// Auto-reload detection on device restart
let consecutiveFailures = 0;
let wasRestarting = false;
setInterval(async () => {
  try {
    const response = await fetch('/status', { method: 'GET', cache: 'no-cache' });
    if (response.ok) {
      if (wasRestarting && consecutiveFailures > 2) {
        console.log('Device back online, reloading page...');
        location.reload();
      }
      consecutiveFailures = 0;
    } else {
      consecutiveFailures++;
    }
  } catch (e) {
    consecutiveFailures++;
    if (consecutiveFailures > 2) {
      wasRestarting = true;
    }
  }
}, 1000);
</script>
</div></body></html>
)HTML";

// -------------------- Advanced Page HTML --------------------
static const char ADVANCED_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>Advanced - HTPC-BLEBoot</title>
<style>
body{font-family:system-ui,Arial;margin:0;background:#111;color:#eee;padding-top:50px}
.topbar{position:fixed;top:0;left:0;right:0;background:#1d1d1d;border-bottom:2px solid #0a84ff;padding:10px 20px;display:flex;align-items:center;gap:15px;z-index:1000;box-shadow:0 2px 8px rgba(0,0,0,0.5)}
.topbar .logo{height:32px;display:flex;align-items:center;margin-right:auto;font-size:16px;font-weight:bold;color:#0a84ff}
.topbar .logo svg{margin-right:8px}
.topbar .btn{background:#0a84ff;border:none;color:#fff;padding:8px 14px;border-radius:6px;cursor:pointer;text-decoration:none;font-size:13px}
.wrap{max-width:720px;margin:0 auto;padding:20px}
.card{background:#1d1d1d;border:1px solid #333;border-radius:8px;padding:16px;margin:16px 0}
.btn{background:#0a84ff;border:none;color:#fff;padding:10px 16px;border-radius:6px;cursor:pointer;text-decoration:none;display:inline-block}
.btn.small{padding:6px 10px;font-size:12px}
.btn.danger{background:#d00}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
label{display:block;margin:8px 0}
input[type="text"],input[type="number"]{width:100%;padding:8px;background:#1d1d1d;color:#eee;border:1px solid #444;border-radius:4px;box-sizing:border-box}
</style></head>
<body>
<div class="topbar">
  <div class="logo">
    <svg width="28" height="28" viewBox="0 0 96 96" xmlns="http://www.w3.org/2000/svg"><rect fill="#0a84ff" x="8" y="8" width="80" height="80" rx="8"/><path fill="#fff" d="M24 32h16v8H24zm24 0h24v8H48zM24 48h48v8H24zm0 16h32v8H24z"/></svg>
    HTPC-BLEBoot
  </div>
  <a href="/" class="btn">Home</a>
  <a href="/mapping" class="btn">Mapping</a>
  <a href="/advanced" class="btn">Advanced</a>
</div>
<div class="wrap">

<div class="card">
  <h3>System Information</h3>
  <p style="color:#888;margin:10px 0">Advanced configuration and diagnostic tools.</p>
  <div id="sysInfo" style="font-family:monospace;font-size:13px;line-height:1.6"></div>
</div>

<div class="card">
  <h3>HID Descriptor Management</h3>
  <p style="color:#888;margin:10px 0;font-size:13px">Manage the persistent HID Report Map descriptor. Save the current descriptor captured from BLE to NVS for boot-time USB initialization, or delete the saved descriptor to revert to default.</p>
  <div class="row">
    <button class="btn" onclick="viewDescriptorStatus()">View Descriptor Status</button>
    <button class="btn" onclick="saveDescriptor()">Save Current Descriptor</button>
    <button class="btn danger" onclick="deleteDescriptor()">Delete Saved Descriptor</button>
  </div>
  <hr style="margin:20px 0;border:none;border-top:1px solid #444">
  <div class="row">
    <button class="btn success" onclick="downloadDescriptor()">Download Descriptor</button>
    <label class="btn" style="cursor:pointer;display:inline-block">
      Upload Descriptor
      <input type="file" id="descriptorFile" accept=".txt" style="display:none" onchange="uploadDescriptor()">
    </label>
  </div>
</div>

<div class="card">
  <h3>Button Hold Time Settings</h3>
  <label>Minimum Hold Time (ms):
    <input type="number" id="inputDelay" value="50" min="2" max="200" placeholder="50">
  </label>
  <p style="color:#888;margin:10px 0;font-size:13px">Minimum time a button must be held before the action triggers (2-200ms). Prevents accidental inputs from brief button bumps. Quick taps shorter than this time will be ignored.</p>
  <button class="btn" onclick="updateInputDelay()">Update Hold Time</button>
</div>

<div class="card">
  <h3>LED Settings</h3>
  <label style="margin:8px 0;display:flex;align-items:center;gap:6px">
    <input type="checkbox" id="connectionLed" checked>
    <span style="font-size:13px">Play rainbow animation on successful connection</span>
  </label>
  <p style="color:#888;margin:10px 0;font-size:13px">When enabled, the onboard LED will display a smooth rainbow animation ending in blue for 3 seconds when a BLE device connects successfully.</p>
  <button class="btn" onclick="updateLedSettings()">Update LED Settings</button>
</div>

<div class="card">
  <h3>Wi-Fi AP Configuration</h3>
  <label>SSID (Network Name):
    <input type="text" id="apSsid" value="HTPC-BLEBoot" placeholder="HTPC-BLEBoot">
  </label>
  <label>Password (leave empty for open network):
    <input type="text" id="apPass" value="" placeholder="(optional)">
  </label>
  <label>mDNS Hostname (access via http://hostname.local):
    <input type="text" id="mdnsHost" value="remote" placeholder="remote">
  </label>
  <p style="color:#888;margin:10px 0;font-size:13px">Access this device at http://hostname.local (default: http://remote.local) in addition to the IP address</p>
  <button class="btn" onclick="updateWiFi()">Update Wi-Fi (requires reboot)</button>
</div>

<div class="card">
  <h3>Backup & Restore</h3>
  <p style="color:#888;margin:10px 0;font-size:13px">Backup all settings (Wi-Fi, button maps, timings, names) to a file. Bluetooth pairing is not included.</p>
  <div class="row">
    <button class="btn" onclick="downloadBackup()">Download Backup</button>
    <button class="btn" onclick="document.getElementById('restoreFile').click()">Restore Backup</button>
  </div>
  <input type="file" id="restoreFile" accept=".json" style="display:none" onchange="restoreBackup(this.files[0])">
  <div id="restoreStatus" style="margin-top:10px;color:#888;font-size:13px"></div>
</div>

<div class="card">
  <h3>System Actions</h3>
  <div class="row">
    <button class="btn" onclick="rebootDevice()">Reboot Device</button>
    <button class="btn danger" onclick="factoryReset()">Factory Reset</button>
  </div>
  <p style="color:#888;margin-top:10px;font-size:13px">Factory Reset will clear all button mappings and saved devices.</p>
</div>

<script>
async function loadInfo(){
  const r = await fetch('/status');
  const s = await r.json();
  
  const info = document.getElementById('sysInfo');
  info.innerHTML = `
    <div>Firmware: HTPC-BLEBoot v1.01</div>
    <div>Connected: ${s.connected ? 'Yes' : 'No'}</div>
    <div>Active Device: ${s.activeName || 'None'}</div>
    <div>Saved Device: ${s.savedName || 'None'}</div>
    <div>Button Maps: ${s.maps ? s.maps.length : 0}</div>
    <div>Min Hold Time: ${s.inputDelay || 50}ms</div>
    <div style="margin-top:12px;padding-top:12px;border-top:1px solid #333">
      <strong>Persistent Storage (NVS):</strong><br>
      Used: ${((s.nvsUsedBytes || 0) / 1024).toFixed(2)} KB (${s.nvsUsedPercent || 0}%)<br>
      Free: ${((s.nvsFreeBytes || 0) / 1024).toFixed(2)} KB<br>
      Total: ${((s.nvsTotalBytes || 0) / 1024).toFixed(2)} KB<br>
      <span style="color:#888;font-size:11px">Entries: ${s.nvsUsedEntries || 0} used / ${s.nvsTotalEntries || 0} total</span>
    </div>
  `;
  
  // Update input delay field
  const delayInput = document.getElementById('inputDelay');
  if(delayInput && s.inputDelay !== undefined) {
    delayInput.value = s.inputDelay;
  }
  
  // Update connection LED checkbox
  const connLedInput = document.getElementById('connectionLed');
  if(connLedInput && s.connectionLed !== undefined) {
    connLedInput.checked = s.connectionLed;
  }
  
  // Update WiFi settings fields
  const ssidInput = document.getElementById('apSsid');
  if(ssidInput && s.apSsid) {
    ssidInput.value = s.apSsid;
  }
  
  const mdnsInput = document.getElementById('mdnsHost');
  if(mdnsInput && s.mdnsHost) {
    mdnsInput.value = s.mdnsHost;
  }
}

function updateInputDelay(){
  const delay = parseInt(document.getElementById('inputDelay').value);
  
  if(isNaN(delay) || delay < 2 || delay > 200){
    alert('Minimum hold time must be between 2 and 200ms');
    return;
  }
  
  fetch('/advanced/delay?ms='+delay)
    .then(r => r.json())
    .then(d => {
      if(d.ok){
        alert('Minimum hold time updated to ' + delay + 'ms');
        loadInfo();
      } else {
        alert('Error: ' + (d.error || d.err || 'unknown'));
      }
    })
    .catch(e => {
      alert('Failed to update: ' + e.message);
    });
}

function updateLedSettings(){
  const enabled = document.getElementById('connectionLed').checked ? 1 : 0;
  
  fetch('/advanced/connled?enabled='+enabled)
    .then(r => r.json())
    .then(d => {
      if(d.ok){
        alert('LED settings updated');
        loadInfo();
      } else {
        alert('Error: ' + (d.error || d.err || 'unknown'));
      }
    })
    .catch(e => {
      alert('Failed to update: ' + e.message);
    });
}

function updateWiFi(){
  const ssid = document.getElementById('apSsid').value;
  const pass = document.getElementById('apPass').value;
  const mdns = document.getElementById('mdnsHost').value;
  
  if(!ssid || ssid.length < 1){
    alert('SSID cannot be empty');
    return;
  }
  
  if(!mdns || mdns.length < 1){
    alert('mDNS hostname cannot be empty');
    return;
  }
  
  if(confirm('Update Wi-Fi settings and reboot?\n\nSSID: ' + ssid + '\nPassword: ' + (pass ? '(set)' : '(open)') + '\nmDNS: http://' + mdns + '.local')){
    fetch('/advanced/wifi?ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)+'&mdns='+encodeURIComponent(mdns))
      .then(r => r.json())
      .then(d => {
        if(d.ok){
          alert('Wi-Fi updated! Device will reboot now.\n\nReconnect to: ' + ssid + '\nAccess at: http://' + mdns + '.local');
        } else {
          alert('Error: ' + (d.err || 'unknown'));
        }
      });
  }
}

function rebootDevice(){
  if(confirm('Reboot the device now?')){
    fetch('/advanced/reboot').then(() => {
      alert('Device is rebooting...');
    });
  }
}

function factoryReset(){
  if(confirm('FACTORY RESET: This will erase ALL button mappings and saved devices!\n\nAre you sure?')){
    if(confirm('This cannot be undone! Continue with factory reset?')){
      fetch('/advanced/factory').then(() => {
        alert('Factory reset complete. Device is rebooting...');
      });
    }
  }
}

async function downloadBackup(){
  try {
    const response = await fetch('/advanced/backup');
    const data = await response.json();
    
    if(!data.ok){
      alert('Error creating backup: ' + (data.error || 'unknown'));
      return;
    }
    
    // Create download
    const blob = new Blob([JSON.stringify(data.backup, null, 2)], {type: 'application/json'});
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'htpc_bleboot_backup_' + new Date().toISOString().split('T')[0] + '.json';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    
    alert('Backup downloaded successfully!');
  } catch(e) {
    alert('Failed to download backup: ' + e.message);
  }
}

async function restoreBackup(file){
  if(!file){
    return;
  }
  
  const statusDiv = document.getElementById('restoreStatus');
  statusDiv.textContent = 'Reading backup file...';
  statusDiv.style.color = '#4a9eff';
  
  try {
    const text = await file.text();
    const backup = JSON.parse(text);
    
    // Validate backup format
    if(!backup.version || !backup.config || !backup.maps){
      throw new Error('Invalid backup file format');
    }
    
    statusDiv.textContent = 'Uploading backup to device...';
    
    const response = await fetch('/advanced/restore', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(backup)
    });
    
    const result = await response.json();
    
    if(result.ok){
      statusDiv.textContent = 'Backup restored successfully! Device will reboot...';
      statusDiv.style.color = '#00ff00';
      setTimeout(() => {
        alert('Settings restored! Device is rebooting.\n\nPlease reconnect to the Wi-Fi network.');
      }, 500);
    } else {
      throw new Error(result.error || 'Restore failed');
    }
  } catch(e) {
    statusDiv.textContent = 'Error: ' + e.message;
    statusDiv.style.color = '#ff4444';
    alert('Failed to restore backup: ' + e.message);
  }
}

loadInfo();
setInterval(loadInfo, 5000);

// Descriptor Management Functions
async function viewDescriptorStatus(){
  try {
    const r = await fetch('/descriptor/status');
    const s = await r.json();
    
    let msg = 'HID Descriptor Status\n\n';
    msg += 'Saved in NVS: ' + (s.hasSaved ? 'YES' : 'NO') + '\n';
    if(s.hasSaved){
      msg += 'Saved Length: ' + s.savedLength + ' bytes\n';
    }
    msg += 'Current Captured: ' + (s.currentLength > 0 ? 'YES' : 'NO') + '\n';
    if(s.currentLength > 0){
      msg += 'Current Length: ' + s.currentLength + ' bytes\n';
    }
    msg += 'USB Started: ' + (s.usbStarted ? 'YES' : 'NO') + '\n';
    
    alert(msg);
  } catch(e) {
    alert('Failed to fetch descriptor status: ' + e.message);
  }
}

async function saveDescriptor(){
  if(!confirm('Save current captured HID descriptor to NVS?\n\nThis will be loaded on next boot for USB initialization.')){
    return;
  }
  
  try {
    const r = await fetch('/descriptor/save');
    const s = await r.json();
    
    if(s.ok){
      alert('Descriptor saved successfully!\n\nLength: ' + s.length + ' bytes\n\nDevice will use this descriptor on next boot.');
    } else {
      alert('Failed to save descriptor: ' + (s.error || 'unknown error'));
    }
  } catch(e) {
    alert('Failed to save descriptor: ' + e.message);
  }
}

async function deleteDescriptor(){
  if(!confirm('Delete saved HID descriptor from NVS?\n\nDevice will revert to default descriptor on next boot.')){
    return;
  }
  
  try {
    const r = await fetch('/descriptor/delete');
    const s = await r.json();
    
    if(s.ok){
      alert('Saved descriptor deleted successfully!\n\nDevice will use default descriptor on next boot.');
    } else {
      alert('Failed to delete descriptor: ' + (s.error || 'unknown error'));
    }
  } catch(e) {
    alert('Failed to delete descriptor: ' + e.message);
  }
}

async function downloadDescriptor(){
  try {
    const r = await fetch('/descriptor/download');
    if(!r.ok){
      const txt = await r.text();
      alert('Failed to download descriptor: ' + txt);
      return;
    }
    
    const blob = await r.blob();
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'hid_descriptor.txt';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    window.URL.revokeObjectURL(url);
    
    alert('Descriptor downloaded successfully!');
  } catch(e) {
    alert('Failed to download descriptor: ' + e.message);
  }
}

async function uploadDescriptor(){
  const fileInput = document.getElementById('descriptorFile');
  const file = fileInput.files[0];
  
  if(!file){
    return;
  }
  
  try {
    const text = await file.text();
    
    const r = await fetch('/descriptor/upload', {
      method: 'POST',
      headers: {'Content-Type': 'text/plain'},
      body: text
    });
    
    const s = await r.json();
    
    if(s.ok){
      alert('Descriptor uploaded successfully!\n\nReboot device to apply the new descriptor.');
      fileInput.value = ''; // Clear file input
    } else {
      alert('Failed to upload descriptor: ' + (s.error || 'unknown error'));
    }
  } catch(e) {
    alert('Failed to upload descriptor: ' + e.message);
  }
}

// Auto-reload detection on device restart
let consecutiveFailures = 0;
let wasRestarting = false;
setInterval(async () => {
  try {
    const response = await fetch('/status', { method: 'GET', cache: 'no-cache' });
    if (response.ok) {
      if (wasRestarting && consecutiveFailures > 2) {
        console.log('Device back online, reloading page...');
        location.reload();
      }
      consecutiveFailures = 0;
    } else {
      consecutiveFailures++;
    }
  } catch (e) {
    consecutiveFailures++;
    if (consecutiveFailures > 2) {
      wasRestarting = true;
    }
  }
}, 1000);
</script>
</div></body></html>
)HTML";

// -------------------- Helpers --------------------
static String bytesToHex(const uint8_t* data, size_t len) {
  String out; out.reserve(len*3);
  for (size_t i=0;i<len;i++) { char b[4]; snprintf(b, sizeof(b), "%02X ", data[i]); out += b; if((i%16)==15) out+="\n"; }
  return out;
}

// -------------------- USB send helpers --------------------
static void usbSendKeyboardReport(uint8_t mod, const uint8_t keys[6]) {
  if (!usbStarted) return;
  uint8_t report[8] = {mod, 0, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]};
  customHID.send(1, report, 8);
  prevMod = mod; 
  memcpy(prevKeys, keys, 6);
}

static void usbSendMouse(uint8_t buttons, int8_t dx, int8_t dy) {
  if (!usbStarted) return;
  uint8_t report[4] = {buttons, (uint8_t)dx, (uint8_t)dy, 0};
  customHID.send(4, report, 4); // Mouse usually uses report ID 4
}

static void usbSendConsumer(uint16_t usage) {
  if (!usbStarted) return;
  uint8_t report[2] = {(uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8)};
  customHID.send(2, report, 2);
}

static void usbSendConsumerRelease() {
  if (!usbStarted) return;
  uint8_t report[2] = {0, 0};
  customHID.send(2, report, 2);
}

static void usbSendConsumerMomentary(uint16_t usage) {
  // For passthrough - press and release immediately
  usbSendConsumer(usage);
  delay(5);
  usbSendConsumerRelease();
}

// -------------------- Mapping --------------------
static void handleButtonAction(int mapIndex, bool pressed) {
  if (mapIndex < 0 || mapIndex >= MAX_BUTTON_MAPS) return;
  ButtonMap* map = &buttonMaps[mapIndex];
  if (!map->active) return;
  
  // Handle button PRESS
  if (pressed) {
    Serial.printf("Button '%s' PRESSED\n", map->label);
    buttonPressStates[mapIndex].isPressed = true;
    buttonPressStates[mapIndex].pressStartTime = millis();
    buttonPressStates[mapIndex].actionTriggered = false;
    usbSentOnce[mapIndex] = false;
    
    // If hold time is NOT enabled, execute action immediately
    if (!map->useHoldTime) {
      // Execute action on press (except GPIO which executes on release)
      if (map->action == 0) {
        // Forward to USB (momentary press)
        usbSendConsumerMomentary(0x0030);
      } else if (map->action == 2) {
        // LED custom color
        pixel.setPixelColor(0, pixel.Color(map->ledR, map->ledG, map->ledB));
        Serial.printf("LED -> RGB(%d,%d,%d)", map->ledR, map->ledG, map->ledB);
        pixel.show();
        
        // Start TTL timer if configured
        if (map->ttlMs > 0) {
          ttlTimers[mapIndex].active = true;
          ttlTimers[mapIndex].startTime = millis();
          ttlTimers[mapIndex].mapIndex = mapIndex;
          Serial.printf(" (TTL: %dms)\n", map->ttlMs);
        } else {
          Serial.println();
        }
      } else if (map->action == 4) {
        // Keyboard remap - send keyboard key
        uint8_t keys[6] = {(uint8_t)(map->keyCode & 0xFF), 0, 0, 0, 0, 0};
        usbSendKeyboardReport(map->keyModifier, keys);
        Serial.printf("Keyboard -> Key 0x%02X (modifier 0x%02X)\n", map->keyCode, map->keyModifier);
      } else if (map->action == 5) {
        // Media control - send consumer report
        usbSendConsumer(map->mediaCode);
        Serial.printf("Media -> Consumer 0x%04X\n", map->mediaCode);
      }
      
      // Handle USB forwarding for actions 1 & 2
      if (map->passToUsb && (map->action == 1 || map->action == 2)) {
        // Extract consumer control code from learned payload (2 bytes, little-endian)
        uint16_t consumerCode = map->payload[0] | (map->payload[1] << 8);
        usbSendConsumerMomentary(consumerCode);
        Serial.printf("  -> Forwarded original button (0x%04X) to USB\n", consumerCode);
      }
    }
  } 
  // Handle button RELEASE
  else {
    Serial.printf("Button '%s' RELEASED\n", map->label);
    unsigned long holdDuration = millis() - buttonPressStates[mapIndex].pressStartTime;
    
    // Reset state BEFORE checking hold time
    buttonPressStates[mapIndex].isPressed = false;
    buttonPressStates[mapIndex].actionTriggered = false;
    
    // Check if hold time requirement was met (if enabled)
    bool holdTimeMet = true;
    if (map->useHoldTime) {
      unsigned long requiredHoldMs = map->holdTimeSec * 1000UL;
      holdTimeMet = (holdDuration >= requiredHoldMs);
      
      if (!holdTimeMet) {
        Serial.printf("  -> Cancelled (held %lums < required %lums)\n", holdDuration, requiredHoldMs);
        
        // Still need to release keyboard/media even if cancelled
        if (map->action == 4) {
          uint8_t keys[6] = {0, 0, 0, 0, 0, 0};
          usbSendKeyboardReport(0, keys);
          Serial.println("  -> Keyboard released (cancelled)");
        } else if (map->action == 5) {
          usbSendConsumerRelease();
          Serial.println("  -> Media released (cancelled)");
        }
        return; // Cancel action - released too early
      } else {
        Serial.printf("  -> Hold time met (%lums >= %lums)\n", holdDuration, requiredHoldMs);
      }
    }
    
    // Execute actions on release based on action type
    if (map->action == 4) {
      // Keyboard: Release on button release
      // (Key press was already sent in loop() when hold time was met)
      uint8_t keys[6] = {0, 0, 0, 0, 0, 0};
      usbSendKeyboardReport(0, keys);
      Serial.println("Keyboard -> Released");
      return;
    }
    
    if (map->action == 5) {
      // Media: Release on button release
      // (Media press was already sent in loop() when hold time was met)
      usbSendConsumerRelease();
      Serial.println("Media -> Released");
      return;
    }
    
    // Execute action on release (if hold time was met or not required)
    if (map->action == 1) {
      // GPIO control
      // If useHoldTime is enabled, GPIO was already executed in loop() when hold time was met
      if (!map->useHoldTime) {
        // Execute GPIO immediately on release (no hold time requirement)
        if (map->gpioMode == 0) {
          // Toggle mode - use persistent toggle state
          gpioToggleStates[mapIndex] = !gpioToggleStates[mapIndex];
          gpioStates[mapIndex] = gpioToggleStates[mapIndex];
          digitalWrite(map->gpioPin, gpioStates[mapIndex] ? HIGH : LOW);
          Serial.printf("GPIO%d TOGGLE -> %s", map->gpioPin, gpioStates[mapIndex] ? "HIGH" : "LOW");
        } else if (map->gpioMode == 1) {
          // Pull HIGH mode
          digitalWrite(map->gpioPin, HIGH);
          gpioStates[mapIndex] = true;
          Serial.printf("GPIO%d -> HIGH", map->gpioPin);
        } else if (map->gpioMode == 2) {
          // Pull LOW mode
          digitalWrite(map->gpioPin, LOW);
          gpioStates[mapIndex] = false;
          Serial.printf("GPIO%d -> LOW", map->gpioPin);
        }
        
        // Start TTL timer if configured
        if (map->ttlMs > 0) {
          ttlTimers[mapIndex].active = true;
          ttlTimers[mapIndex].startTime = millis();
          ttlTimers[mapIndex].mapIndex = mapIndex;
          Serial.printf(" (TTL: %dms)\n", map->ttlMs);
        } else {
          Serial.println();
        }
      }
      
      // Handle USB forwarding
      if (map->passToUsb) {
        // Extract consumer control code from learned payload
        uint16_t consumerCode = map->payload[0] | (map->payload[1] << 8);
        
        if (map->useHoldTime) {
          // With hold time enabled - already sent in loop()
          // Nothing to do here
        } else {
          // Without hold time - send on release
          usbSendConsumerMomentary(consumerCode);
          Serial.printf("  -> Forwarded original button (0x%04X) to USB\n", consumerCode);
        }
      }
    } else if (map->action == 2) {
      // LED action on release
      if (map->useHoldTime) {
        // If hold time was required and met, turn LED on now
        pixel.setPixelColor(0, pixel.Color(map->ledR, map->ledG, map->ledB));
        Serial.printf("LED -> RGB(%d,%d,%d)", map->ledR, map->ledG, map->ledB);
        pixel.show();
        
        // Start TTL timer if configured
        if (map->ttlMs > 0) {
          ttlTimers[mapIndex].active = true;
          ttlTimers[mapIndex].startTime = millis();
          ttlTimers[mapIndex].mapIndex = mapIndex;
          Serial.printf(" (TTL: %dms)\n", map->ttlMs);
        } else {
          Serial.println();
        }
        
        // Handle USB forwarding
        if (map->passToUsb) {
          // Extract consumer control code from learned payload
          uint16_t consumerCode = map->payload[0] | (map->payload[1] << 8);
          
          if (map->applyToUsb) {
            // Single send - send once only
            if (!usbSentOnce[mapIndex]) {
              usbSendConsumerMomentary(consumerCode);
              usbSentOnce[mapIndex] = true;
              Serial.printf("  -> Forwarded original button (0x%04X) to USB [SINGLE]\n", consumerCode);
            }
          } else {
            // Repeat mode - already being sent in loop()
          }
        }
      } else {
        // No hold time - turn off on release (unless TTL is handling it)
        if (map->ttlMs == 0) {
          pixel.setPixelColor(0, pixel.Color(0, 0, 0));
          Serial.println("LED -> OFF");
          pixel.show();
        }
      }
    }
  }
}

static void handleConsumerUsage(uint16_t usage) {
  // Always forward consumer controls to USB (momentary)
  usbSendConsumerMomentary(usage);
}

// -------------------- BLE notification routing --------------------
static void notifyCB(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool notify) {
  if (len==0) return;
  
  // Ignore all input if not connected (prevents random USB commands after disconnect)
  if (!isConnected) {
    return;
  }
  
  // Find the Report ID for this characteristic
  uint8_t knownReportId = 0;
  for (auto &rc : inputReports) {
    if (rc.chr == chr) {
      knownReportId = rc.reportId;
      break;
    }
  }
  
  // Debug: print raw data with report ID
  Serial.printf("[ID=%u len=%u] ", knownReportId, len);
  for (size_t i=0; i<len && i<16; i++) Serial.printf("%02X ", data[i]);
  Serial.println();
  
  // LEARN MODE: Capture button press (ignore mouse movement)
  if (learnMode && learnMapIndex >= 0 && learnMapIndex < MAX_BUTTON_MAPS) {
    // Check if learn mode has timed out
    if (millis() - learnModeStartTime > LEARN_TIMEOUT) {
      Serial.println("Learn mode timed out");
      learnMode = false;
      learnMapIndex = -1;
      return;
    }
    
    // Ignore mouse movement data (Report ID 3 or 4 with length >= 3)
    if ((knownReportId == 3 || knownReportId == 4) && len >= 3) {
      // Check if this is mouse movement (has dx/dy values)
      int8_t dx = (int8_t)data[1];
      int8_t dy = (int8_t)data[2];
      if (dx != 0 || dy != 0) {
        Serial.println("  -> Ignoring mouse movement in learn mode");
        return;
      }
    }
    
    // Ignore keyboard modifier-only or empty reports
    if (knownReportId == 1 && len == 8) {
      bool hasKeys = false;
      for (int i = 2; i < 8; i++) {
        if (data[i] != 0) {
          hasKeys = true;
          break;
        }
      }
      if (!hasKeys) {
        Serial.println("  -> Ignoring keyboard modifier-only in learn mode");
        return;
      }
    }
    
    // Valid button press detected - learn it!
    Serial.printf(">>> LEARNED BUTTON for map #%d!\n", learnMapIndex);
    Serial.printf("    Report ID: %u, Length: %u\n", knownReportId, len);
    Serial.print("    Payload: ");
    for (size_t i = 0; i < len && i < 8; i++) Serial.printf("%02X ", data[i]);
    Serial.println();
    
    buttonMaps[learnMapIndex].reportId = knownReportId;
    buttonMaps[learnMapIndex].payloadLen = len;
    memset(buttonMaps[learnMapIndex].payload, 0, sizeof(buttonMaps[learnMapIndex].payload));
    memcpy(buttonMaps[learnMapIndex].payload, data, len < 8 ? len : 8);
    
    // Save this specific button map to preferences
    prefs.begin("maps", false);
    char key[16];
    snprintf(key, sizeof(key), "active%d", learnMapIndex);
    prefs.putBool(key, buttonMaps[learnMapIndex].active);
    snprintf(key, sizeof(key), "label%d", learnMapIndex);
    prefs.putString(key, buttonMaps[learnMapIndex].label);
    snprintf(key, sizeof(key), "rid%d", learnMapIndex);
    prefs.putUChar(key, buttonMaps[learnMapIndex].reportId);
    snprintf(key, sizeof(key), "plen%d", learnMapIndex);
    prefs.putUChar(key, buttonMaps[learnMapIndex].payloadLen);
    snprintf(key, sizeof(key), "pay%d", learnMapIndex);
    prefs.putBytes(key, buttonMaps[learnMapIndex].payload, 8);
    snprintf(key, sizeof(key), "act%d", learnMapIndex);
    prefs.putUChar(key, buttonMaps[learnMapIndex].action);
    snprintf(key, sizeof(key), "gpio%d", learnMapIndex);
    prefs.putUChar(key, buttonMaps[learnMapIndex].gpioPin);
    snprintf(key, sizeof(key), "gpioMode%d", learnMapIndex);
    prefs.putUChar(key, buttonMaps[learnMapIndex].gpioMode);
    snprintf(key, sizeof(key), "gpioTtlBeh%d", learnMapIndex);
    prefs.putUChar(key, buttonMaps[learnMapIndex].gpioTtlBehavior);
    snprintf(key, sizeof(key), "ttl%d", learnMapIndex);
    prefs.putUShort(key, buttonMaps[learnMapIndex].ttlMs);
    snprintf(key, sizeof(key), "pass%d", learnMapIndex);
    prefs.putBool(key, buttonMaps[learnMapIndex].passToUsb);
    snprintf(key, sizeof(key), "ledR%d", learnMapIndex);
    prefs.putUChar(key, buttonMaps[learnMapIndex].ledR);
    snprintf(key, sizeof(key), "ledG%d", learnMapIndex);
    prefs.putUChar(key, buttonMaps[learnMapIndex].ledG);
    snprintf(key, sizeof(key), "ledB%d", learnMapIndex);
    prefs.putUChar(key, buttonMaps[learnMapIndex].ledB);
    prefs.end();
    
    learnMode = false;
    learnMapIndex = -1;
    Serial.println("Learn mode complete - button saved!");
    return;
  }
  
  // Most HID devices don't include the report ID in the data itself when using
  // separate characteristics per report. Use the full data as payload.
  const uint8_t* payload = data;
  size_t payLen = len;
  
  // Check all active button maps for matching patterns
  for (int i = 0; i < MAX_BUTTON_MAPS; i++) {
    if (!buttonMaps[i].active) continue;
    
    // Check if this matches the learned button (PRESS)
    if (knownReportId == buttonMaps[i].reportId && payLen == buttonMaps[i].payloadLen) {
      bool matches = true;
      for (size_t j = 0; j < payLen && j < 8; j++) {
        if (payload[j] != buttonMaps[i].payload[j]) {
          matches = false;
          break;
        }
      }
      if (matches) {
        // Button PRESSED - start tracking hold time
        if (!buttonPressStates[i].isPressed) {
          // First press detected
          buttonPressStates[i].isPressed = true;
          buttonPressStates[i].pressStartTime = millis();
          buttonPressStates[i].actionTriggered = false;
          
          // Only show global input delay tracking if per-button hold time is NOT enabled
          if (!buttonMaps[i].useHoldTime) {
            Serial.printf("Button #%d pressed, tracking hold time (need %dms)...\n", i, inputDelayMs);
          }
        }
        return; // Don't process further - this is a mapped button
      }
    }
    
    // Check for button RELEASE (same Report ID, same length, but all zeros)
    if (knownReportId == buttonMaps[i].reportId && payLen == buttonMaps[i].payloadLen) {
      bool allZero = true;
      for (size_t j = 0; j < payLen && j < 8; j++) {
        if (payload[j] != 0x00) {
          allZero = false;
          break;
        }
      }
      if (allZero && buttonPressStates[i].isPressed) {
        // Button RELEASED - check if held long enough
        unsigned long holdTime = millis() - buttonPressStates[i].pressStartTime;
        Serial.printf("Button #%d released after %lums\n", i, holdTime);
        
        // Check if button was held long enough to trigger action
        if (holdTime >= inputDelayMs) {
          Serial.printf("  -> Hold time sufficient (%lums >= %dms) - executing action!\n", holdTime, inputDelayMs);
          handleButtonAction(i, true);  // Press action
          handleButtonAction(i, false); // Release action
        } else {
          Serial.printf("  -> Button released too quickly (%lums < %dms) - ignoring\n", holdTime, inputDelayMs);
        }
        
        // Reset state
        buttonPressStates[i].isPressed = false;
        buttonPressStates[i].actionTriggered = false;
        return; // Don't process further - this is a mapped button
      }
    }
  }
  
  // If no button mapping matched, forward raw HID report to USB
  if (usbStarted) {
    customHID.send(knownReportId, payload, payLen);
  }
}

// Read battery level from Battery Service (0x180F)
static void readBatteryLevel(NimBLEClient* c) {
  batteryLevel = -1; // Reset to unknown
  
  NimBLERemoteService* pBattery = c->getService(NimBLEUUID((uint16_t)0x180F));
  if (!pBattery) {
    Serial.println("Battery Service (0x180F) not found");
    return;
  }
  
  NimBLERemoteCharacteristic* pBatteryChar = pBattery->getCharacteristic(NimBLEUUID((uint16_t)0x2A19));
  if (!pBatteryChar) {
    Serial.println("Battery Level characteristic (0x2A19) not found");
    return;
  }
  
  if (!pBatteryChar->canRead()) {
    Serial.println("Battery Level characteristic cannot be read");
    return;
  }
  
  std::string val = pBatteryChar->readValue();
  Serial.printf("Battery raw value length: %d\n", val.length());
  if (val.length() > 0) {
    batteryLevel = (uint8_t)val[0];
    Serial.printf("Battery Level: %d%% (raw byte: 0x%02X)\n", batteryLevel, (uint8_t)val[0]);
    
    // Print all bytes for debugging
    if (val.length() > 1) {
      Serial.print("Additional bytes: ");
      for (size_t i = 1; i < val.length(); i++) {
        Serial.printf("0x%02X ", (uint8_t)val[i]);
      }
      Serial.println();
    }
  } else {
    Serial.println("Battery value is empty");
  }
}

// Discover HID service, subscribe input reports, read report map
static bool setupHID(NimBLEClient* c) {
  pHID = c->getService(NimBLEUUID((uint16_t)0x1812));
  if (!pHID) { Serial.println("HID service (0x1812) not found"); return false; }

  // Read Report Map and capture raw descriptor bytes
  NimBLERemoteCharacteristic* mapChr = pHID->getCharacteristic(NimBLEUUID((uint16_t)0x2A4B));
  hidMapHex = "";
  capturedDescriptorLen = 0;
  
  if (mapChr && mapChr->canRead()) {
    std::string val = mapChr->readValue();
    capturedDescriptorLen = val.size();
    
    if (capturedDescriptorLen > 0 && capturedDescriptorLen <= sizeof(capturedDescriptor)) {
      memcpy(capturedDescriptor, val.data(), capturedDescriptorLen);
      hidMapHex = bytesToHex((const uint8_t*)val.data(), val.size());
      
      Serial.printf("Captured %u-byte HID descriptor from BLE device\n", capturedDescriptorLen);
      Serial.println("HID Descriptor (hex):");
      for (uint16_t i = 0; i < capturedDescriptorLen; i++) {
        Serial.printf("%02X ", capturedDescriptor[i]);
        if ((i + 1) % 16 == 0) Serial.println();
      }
      Serial.println();
      
      // Initialize USB HID with captured descriptor
      if (!usbStarted) {
        Serial.println("Initializing USB HID with captured descriptor...");
        customHID.begin(capturedDescriptor, capturedDescriptorLen);
        USB.begin();
        usbStarted = true;
        Serial.println("USB HID started with custom descriptor");
      }
    } else {
      Serial.printf("ERROR: Invalid HID descriptor length: %d\n", capturedDescriptorLen);
    }
  } else {
    Serial.println("ERROR: Cannot read HID Report Map characteristic");
    return false;
  }

  inputReports.clear();

  // Iterate all characteristics and collect 0x2A4D (Report)
  auto chrs = pHID->getCharacteristics(true);
  if (chrs) {
    for (auto* chr : *chrs) {
    if (chr->getUUID().equals(NimBLEUUID((uint16_t)0x2A4D))) {
      ReportChar rc; rc.chr = chr; rc.reportId = 0; rc.reportType = 0;
      // Read Report Reference descriptor (0x2908): [reportId, reportType]
      NimBLERemoteDescriptor* ref = chr->getDescriptor(NimBLEUUID((uint16_t)0x2908));
      if (ref) {
        std::string dv = ref->readValue();
        if (dv.size() >= 2) { rc.reportId = (uint8_t)dv[0]; rc.reportType=(uint8_t)dv[1]; }
      }
      if (rc.reportType == 1 /*Input*/ && (chr->canNotify() || chr->canIndicate())) {
        if (chr->subscribe(true, notifyCB)) {
          inputReports.push_back(rc);
          Serial.printf("Subscribed input report (ID=%u)\n", rc.reportId);
        }
      }
    }
    }
  }
  return !inputReports.empty();
}

// Forward declaration
static void startConnectTask(const String& addr, const String& name);

// Connect to remote by address (background task)
class MyClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* c) override {
    Serial.println("BLE connected, starting pairing...");
    // Now that we're connected, request pairing
    if (!c->secureConnection()) {
      Serial.println("ERROR: Failed to initiate pairing");
      c->disconnect();
    }
  }
  
  void onDisconnect(NimBLEClient* c) override {
    Serial.println("Disconnected");
    isConnected = false;
    isConnecting = false;
    connectRunning = false;
    activeAddr = "";
    activeName = "";
    batteryLevel = -1; // Reset battery level on disconnect
    
    // Release all USB keys/buttons to prevent stuck keys
    if (usbStarted) {
      uint8_t emptyReport[8] = {0};
      customHID.send(1, emptyReport, 8); // Release keyboard
      customHID.send(2, emptyReport, 2); // Release consumer
      customHID.send(4, emptyReport, 4); // Release mouse
      Serial.println("USB HID reports cleared");
    }
  }
  
  uint32_t onPassKeyRequest() override {
    Serial.printf("Pairing passkey: %u\n", passkey);
    return passkey;
  }
  
  void onAuthenticationComplete(ble_gap_conn_desc* desc) override {
    if (!desc) return;
    Serial.printf("Auth complete for %s; bonded=%d encrypted=%d authenticated=%d\n", 
                  NimBLEAddress(desc->peer_ota_addr).toString().c_str(), 
                  desc->sec_state.bonded,
                  desc->sec_state.encrypted,
                  desc->sec_state.authenticated);
    
    // CRITICAL: Wait for BOTH bonding AND encryption before proceeding
    if (desc->sec_state.bonded && desc->sec_state.encrypted) {
      Serial.println("Bonding and encryption complete - pairing successful!");
      pairingComplete = true;
    } else if (desc->sec_state.bonded && !desc->sec_state.encrypted) {
      Serial.println("WARNING: Bonded but not encrypted yet, waiting for encryption...");
    } else if (!desc->sec_state.bonded && desc->sec_state.encrypted) {
      Serial.println("WARNING: Encrypted but not bonded - unusual state");
    } else {
      Serial.println("WARNING: Neither bonded nor encrypted - pairing may have failed");
    }
  }
  
  bool onConfirmPIN(uint32_t pin) override {
    Serial.printf("Confirm pairing PIN: %u => YES\n", pin);
    return true;
  }
};

static void startConnectTask(const String& addr, const String& name) {
  if (connectRunning) return;
  pendingConnectAddr = addr;
  pendingConnectName = name;
  connectRunning = true;
  isConnecting = true;
  
  if (connectTaskHandle) { vTaskDelete(connectTaskHandle); connectTaskHandle = nullptr; }
  
  xTaskCreatePinnedToCore(
    [](void* p){
      String addr = pendingConnectAddr;
      String name = pendingConnectName;
      
      if (!pClient) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCallbacks());
      } else if (pClient->isConnected()) {
        pClient->disconnect(); 
        delay(100);
      }

      NimBLEAddress address(addr.c_str());
      
      // Save the target device info for callbacks
      savedAddr = addr;
      savedName = name;
      activeAddr = addr;
      activeName = name;
      
      // Try multiple connection attempts
      bool connected = false;
      for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("Connect attempt %d/3 to %s\n", attempt, addr.c_str());
        
        if (pClient->connect(address)) {
          connected = true;
          Serial.println("BLE connection established, waiting for callbacks...");
          break;
        }
        
        if (attempt < 3) {
          Serial.println("Retry...");
          delay(800);
        }
      }
      
      if (!connected) { 
        Serial.println("Connect failed after retries"); 
        isConnecting = false;
        connectRunning = false;
        activeAddr = "";
        activeName = "";
        vTaskDelete(nullptr);
        return;
      }

      Serial.println("Connected successfully");
      
      // Check if already bonded to this device by checking bond count
      int bondCount = NimBLEDevice::getNumBonds();
      bool alreadyBonded = (bondCount > 0);
      
      if (alreadyBonded) {
        Serial.printf("Found %d existing bond(s) - skipping pairing\n", bondCount);
      }
      
      bool justPaired = false;
      if (!alreadyBonded) {
        Serial.println("Initiating pairing...");
        // CRITICAL: Initiate pairing/bonding on FIRST connection only
        if (!pClient->secureConnection()) {
          Serial.println("Pairing failed");
          if (pClient && pClient->isConnected()) pClient->disconnect();
          isConnecting = false;
          connectRunning = false;
          vTaskDelete(nullptr);
          return;
        }
        Serial.println("Pairing successful");
        justPaired = true;
        // Give time for bonding to complete on first pairing
        delay(1000);
      } else {
        // Already bonded - just wait a bit for connection to stabilize
        Serial.println("Using existing bond");
        delay(500);
      }
      
      Serial.println("Setting up HID...");

      bool hidOk = setupHID(pClient);
      if (hidOk) {
        activeAddr = addr; activeName = name;
        savedAddr = addr; savedName = name;
        
        // Read battery level (if available)
        readBatteryLevel(pClient);
        lastBatteryRead = millis(); // Start the battery polling timer
        
        // Save to preferences immediately
        prefs.begin("cfg", false);
        prefs.putString("remoteAddr", savedAddr);
        prefs.putString("remoteName", savedName);
        prefs.end();
        
        Serial.println("HID setup complete");
        
        // If we just paired for the first time, disconnect and reboot immediately
        if (justPaired) {
          Serial.println("First pairing complete - disconnecting and rebooting for clean reconnect...");
          delay(500);
          if (pClient && pClient->isConnected()) {
            pClient->disconnect();
          }
          delay(500);
          ESP.restart();
        }
        
        // Normal connection - mark as connected
        isConnected = true;
        Serial.println("Maintaining connection...");
        
        // Play connection animation
        playConnectionAnimation();
      } else {
        Serial.println("HID setup failed; disconnecting");
        if (pClient && pClient->isConnected()) pClient->disconnect();
        activeAddr = ""; 
        activeName = "";
        isConnected = false;
      }
      
      // Wait before cleanup to avoid race with disconnect callback
      isConnecting = false;
      connectRunning = false;
      delay(500);
      vTaskDelete(nullptr);
    },
    "connectTask", 8192, nullptr, 1, &connectTaskHandle, 1
  );
}

// -------------------- HTTP handlers --------------------
static void handleStatus(AsyncWebServerRequest* req) {
  // Refresh battery level if connected (rate limited to max 5 times per minute)
  if (isConnected && pClient && pClient->isConnected()) {
    unsigned long now = millis();
    if (now - lastBatteryRead >= BATTERY_MIN_INTERVAL) {
      readBatteryLevel(pClient);
      lastBatteryRead = now;
    }
  }
  
  String json = "{";
  json += "\"connected\":"; json += isConnected?"true":"false"; json += ",";
  json += "\"connecting\":"; json += isConnecting?"true":"false"; json += ",";
  json += "\"learning\":"; json += learnMode?"true":"false"; json += ",";
  json += "\"activeAddr\":\"" + activeAddr + "\",";
  json += "\"activeName\":\"" + activeName + "\",";
  json += "\"savedAddr\":\"" + savedAddr + "\",";
  json += "\"savedName\":\"" + savedName + "\",";
  
  // Load WiFi settings for UI
  prefs.begin("cfg", true);
  String apSsid = prefs.getString("apSsid", "HTPC-BLEBoot");
  String mdnsHost = prefs.getString("mdnsHost", "remote");
  prefs.end();
  
  json += "\"apSsid\":\"" + apSsid + "\",";
  json += "\"mdnsHost\":\"" + mdnsHost + "\",";
  
  // Send all active button maps
  json += "\"maps\":[";
  bool first = true;
  for (int i = 0; i < MAX_BUTTON_MAPS; i++) {
    if (!buttonMaps[i].active) continue;
    if (!first) json += ",";
    first = false;
    
    json += "{\"idx\":" + String(i) + ",";
    json += "\"label\":\"" + String(buttonMaps[i].label) + "\",";
    json += "\"action\":" + String(buttonMaps[i].action) + ",";
    json += "\"gpioPin\":" + String(buttonMaps[i].gpioPin) + ",";
    json += "\"gpioMode\":" + String(buttonMaps[i].gpioMode) + ",";
    json += "\"gpioTtlBehavior\":" + String(buttonMaps[i].gpioTtlBehavior) + ",";
    json += "\"ttlMs\":" + String(buttonMaps[i].ttlMs) + ",";
    json += "\"passToUsb\":" + String(buttonMaps[i].passToUsb?"true":"false") + ",";
    json += "\"useHoldTime\":" + String(buttonMaps[i].useHoldTime?"true":"false") + ",";
    json += "\"holdTimeSec\":" + String(buttonMaps[i].holdTimeSec) + ",";
    json += "\"applyToUsb\":" + String(buttonMaps[i].applyToUsb?"true":"false") + ",";
    json += "\"ledR\":" + String(buttonMaps[i].ledR) + ",";
    json += "\"ledG\":" + String(buttonMaps[i].ledG) + ",";
    json += "\"ledB\":" + String(buttonMaps[i].ledB) + "}";
  }
  json += "],";
  
  // Escape newlines in hidMapHex
  String m = hidMapHex; m.replace("\n","\\n");
  json += "\"hidMap\":\"" + m + "\",";
  json += "\"inputDelay\":" + String(inputDelayMs) + ",";
  json += "\"connectionLed\":" + String(connectionLedEnabled?"true":"false") + ",";
  json += "\"batteryLevel\":" + String(batteryLevel) + ",";
  
  // Add NVS storage statistics
  nvs_stats_t nvs_stats;
  if (nvs_get_stats(NULL, &nvs_stats) == ESP_OK) {
    size_t used_bytes = nvs_stats.used_entries * 32;
    size_t free_bytes = nvs_stats.free_entries * 32;
    size_t total_bytes = nvs_stats.total_entries * 32;
    float used_pct = (float)used_bytes / total_bytes * 100.0;
    
    json += "\"nvsUsedEntries\":" + String(nvs_stats.used_entries) + ",";
    json += "\"nvsFreeEntries\":" + String(nvs_stats.free_entries) + ",";
    json += "\"nvsTotalEntries\":" + String(nvs_stats.total_entries) + ",";
    json += "\"nvsUsedBytes\":" + String(used_bytes) + ",";
    json += "\"nvsFreeBytes\":" + String(free_bytes) + ",";
    json += "\"nvsTotalBytes\":" + String(total_bytes) + ",";
    json += "\"nvsUsedPercent\":" + String(used_pct, 1);
  } else {
    json += "\"nvsUsedEntries\":0,";
    json += "\"nvsFreeEntries\":0,";
    json += "\"nvsTotalEntries\":0,";
    json += "\"nvsUsedBytes\":0,";
    json += "\"nvsFreeBytes\":0,";
    json += "\"nvsTotalBytes\":0,";
    json += "\"nvsUsedPercent\":0";
  }
  
  json += "}";
  req->send(200, "application/json", json);
}

// Helper function to extract JSON value by key
String getValue(const String& json, const String& key) {
  int keyStart = json.indexOf("\"" + key + "\":");
  if(keyStart == -1) return "";
  
  int valueStart = json.indexOf(":", keyStart) + 1;
  
  // Skip whitespace
  while(valueStart < json.length() && (json.charAt(valueStart) == ' ' || json.charAt(valueStart) == '\n'))
    valueStart++;
  
  // Check if value is string (starts with quote)
  if(json.charAt(valueStart) == '"'){
    valueStart++; // Skip opening quote
    int valueEnd = json.indexOf("\"", valueStart);
    return json.substring(valueStart, valueEnd);
  }
  
  // Numeric or boolean value
  int valueEnd = valueStart;
  while(valueEnd < json.length() && json.charAt(valueEnd) != ',' && json.charAt(valueEnd) != '}')
    valueEnd++;
  
  return json.substring(valueStart, valueEnd);
}

// Rainbow LED animation for successful connection
void playConnectionAnimation() {
  if (!connectionLedEnabled) return;
  
  const int duration = 3000; // 3 seconds
  const int steps = 60; // 60 steps for smooth animation
  const int delayMs = duration / steps;
  
  for (int i = 0; i < steps; i++) {
    float phase = (float)i / steps;
    
    uint8_t r, g, b;
    if (phase < 0.85) {
      // Rainbow phase (0 to 85%)
      float rainbowPhase = phase / 0.85; // Normalize to 0-1
      int hue = (int)(rainbowPhase * 255);
      
      // HSV to RGB conversion (simplified)
      int region = hue / 43;
      int remainder = (hue - (region * 43)) * 6;
      
      switch (region) {
        case 0: r = 255; g = remainder; b = 0; break;
        case 1: r = 255 - remainder; g = 255; b = 0; break;
        case 2: r = 0; g = 255; b = remainder; break;
        case 3: r = 0; g = 255 - remainder; b = 255; break;
        case 4: r = remainder; g = 0; b = 255; break;
        default: r = 255; g = 0; b = 255 - remainder; break;
      }
    } else {
      // Fade to blue phase (85% to 100%)
      float bluePhase = (phase - 0.85) / 0.15; // Normalize to 0-1
      r = (uint8_t)(255 * (1 - bluePhase));
      g = 0;
      b = 255;
    }
    
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
    delay(delayMs);
  }
  
  // Final blue color
  pixel.setPixelColor(0, pixel.Color(0, 0, 255));
  pixel.show();
  delay(500);
  
  // Turn off
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();
}

// Slow red pulsing LED when no device is paired
void updateNoPairPulse() {
  // Only pulse if no device is saved
  if (savedAddr.length() > 0) {
    // Device is saved, ensure LED is off if it was pulsing
    if (noPairPulseActive) {
      pixel.setPixelColor(0, pixel.Color(0, 0, 0));
      pixel.show();
      noPairPulseActive = false;
    }
    return;
  }
  
  // Start the pulse tracking
  if (!noPairPulseActive) {
    noPairPulseStart = millis();
    noPairPulseActive = true;
  }
  
  // Slow pulse: 3 second cycle (1.5s fade in, 1.5s fade out)
  unsigned long elapsed = millis() - noPairPulseStart;
  unsigned long cycleTime = 3000; // 3 second full cycle
  unsigned long phase = elapsed % cycleTime;
  
  // Calculate brightness (0-76 for 30% of 255)
  float brightness;
  if (phase < cycleTime / 2) {
    // Fade in (0 to 1.5s)
    brightness = (float)phase / (cycleTime / 2);
  } else {
    // Fade out (1.5s to 3s)
    brightness = 1.0 - ((float)(phase - cycleTime / 2) / (cycleTime / 2));
  }
  
  // Apply smooth easing (sine wave for smoother fade)
  brightness = (sin(brightness * PI - PI/2) + 1) / 2;
  
  uint8_t red = (uint8_t)(brightness * 76); // 30% of 255 = 76
  pixel.setPixelColor(0, pixel.Color(red, 0, 0));
  pixel.show();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  
  bootTime = millis();
  
  // Create mutex for discovered devices vector
  discoveredMutex = xSemaphoreCreateMutex();

  // Load saved remote and Wi-Fi settings
  prefs.begin("cfg", true);
  savedAddr = prefs.getString("remoteAddr", "");
  savedName = prefs.getString("remoteName", "");
  String apSsid = prefs.getString("apSsid", "HTPC-BLEBoot");
  String apPass = prefs.getString("apPass", "");
  String mdnsHost = prefs.getString("mdnsHost", "remote");
  inputDelayMs = prefs.getUChar("inputDelay", 50);
  connectionLedEnabled = prefs.getBool("connLed", true);
  prefs.end();
  
  // Load all button maps from preferences
  prefs.begin("maps", true);
  activeButtonMaps = 0;
  for (int i = 0; i < MAX_BUTTON_MAPS; i++) {
    char key[16];
    snprintf(key, sizeof(key), "active%d", i);
    buttonMaps[i].active = prefs.getBool(key, false);
    
    if (buttonMaps[i].active) {
      activeButtonMaps++;
      snprintf(key, sizeof(key), "label%d", i);
      String label = prefs.getString(key, "Button");
      strncpy(buttonMaps[i].label, label.c_str(), sizeof(buttonMaps[i].label) - 1);
      buttonMaps[i].label[sizeof(buttonMaps[i].label) - 1] = '\0';
      
      snprintf(key, sizeof(key), "rid%d", i);
      buttonMaps[i].reportId = prefs.getUChar(key, 3);
      snprintf(key, sizeof(key), "plen%d", i);
      buttonMaps[i].payloadLen = prefs.getUChar(key, 1);
      snprintf(key, sizeof(key), "pay%d", i);
      prefs.getBytes(key, buttonMaps[i].payload, 8);
      snprintf(key, sizeof(key), "act%d", i);
      buttonMaps[i].action = prefs.getUChar(key, 2);
      snprintf(key, sizeof(key), "gpio%d", i);
      buttonMaps[i].gpioPin = prefs.getUChar(key, 2);
      snprintf(key, sizeof(key), "gpioMode%d", i);
      buttonMaps[i].gpioMode = prefs.getUChar(key, 0);
      snprintf(key, sizeof(key), "gpioTtlBeh%d", i);
      buttonMaps[i].gpioTtlBehavior = prefs.getUChar(key, 0);
      snprintf(key, sizeof(key), "ttl%d", i);
      buttonMaps[i].ttlMs = prefs.getUShort(key, 0);
      snprintf(key, sizeof(key), "pass%d", i);
      buttonMaps[i].passToUsb = prefs.getBool(key, false);
      snprintf(key, sizeof(key), "ledR%d", i);
      buttonMaps[i].ledR = prefs.getUChar(key, 0);
      snprintf(key, sizeof(key), "ledG%d", i);
      buttonMaps[i].ledG = prefs.getUChar(key, 255);
      snprintf(key, sizeof(key), "ledB%d", i);
      buttonMaps[i].ledB = prefs.getUChar(key, 0);
      snprintf(key, sizeof(key), "keyCode%d", i);
      buttonMaps[i].keyCode = prefs.getUShort(key, 0x2A); // Default: Backspace
      snprintf(key, sizeof(key), "keyMod%d", i);
      buttonMaps[i].keyModifier = prefs.getUChar(key, 0x00); // Default: No modifiers
      snprintf(key, sizeof(key), "mediaCode%d", i);
      buttonMaps[i].mediaCode = prefs.getUShort(key, 0x00E9); // Default: Volume Up
      snprintf(key, sizeof(key), "useHold%d", i);
      buttonMaps[i].useHoldTime = prefs.getBool(key, false);
      snprintf(key, sizeof(key), "holdSec%d", i);
      buttonMaps[i].holdTimeSec = prefs.getUChar(key, 3);
      snprintf(key, sizeof(key), "applyUsb%d", i);
      buttonMaps[i].applyToUsb = prefs.getBool(key, false);
      
      Serial.printf("Loaded button map #%d: '%s' ID=%u Len=%u RGB(%d,%d,%d)\n", 
                    i, buttonMaps[i].label, buttonMaps[i].reportId, buttonMaps[i].payloadLen,
                    buttonMaps[i].ledR, buttonMaps[i].ledG, buttonMaps[i].ledB);
      
      // Setup GPIO pin if needed
      if (buttonMaps[i].action == 1 && buttonMaps[i].gpioPin < 49) {
        pinMode(buttonMaps[i].gpioPin, OUTPUT);
        digitalWrite(buttonMaps[i].gpioPin, LOW);
      }
    }
  }
  prefs.end();
  
  Serial.printf("Loaded %d active button maps\n", activeButtonMaps);
  
  // Check NVS storage statistics
  nvs_stats_t nvs_stats;
  if (nvs_get_stats(NULL, &nvs_stats) == ESP_OK) {
    Serial.println("\n=== NVS Storage Statistics ===");
    Serial.printf("Used entries:      %d\n", nvs_stats.used_entries);
    Serial.printf("Free entries:      %d\n", nvs_stats.free_entries);
    Serial.printf("Total entries:     %d\n", nvs_stats.total_entries);
    Serial.printf("Namespace count:   %d\n", nvs_stats.namespace_count);
    
    size_t used_bytes = nvs_stats.used_entries * 32; // Each entry is ~32 bytes
    size_t free_bytes = nvs_stats.free_entries * 32;
    size_t total_bytes = nvs_stats.total_entries * 32;
    
    Serial.printf("\nEstimated usage:\n");
    Serial.printf("Used:  %d bytes (%.1f%%)\n", used_bytes, (float)used_bytes / total_bytes * 100.0);
    Serial.printf("Free:  %d bytes (%.1f%%)\n", free_bytes, (float)free_bytes / total_bytes * 100.0);
    Serial.printf("Total: %d bytes\n", total_bytes);
    Serial.println("==============================\n");
  } else {
    Serial.println("Failed to get NVS stats");
  }
  
  // Init LED
  pixel.begin();
  pixel.setBrightness(50);
  pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  pixel.show();

  // ===== EARLY USB INITIALIZATION WITH SAVED DESCRIPTOR =====
  // Load saved HID descriptor from NVS (if it exists)
  if (loadSavedDescriptor()) {
    // Initialize USB with the saved descriptor BEFORE anything else
    Serial.println("Initializing USB HID with saved descriptor...");
    
    // IMPORTANT: Register the HID device BEFORE calling USB.begin()
    customHID.begin(capturedDescriptor, capturedDescriptorLen);
    
    // NOW start the USB stack
    USB.begin();
    
    // Wait for HID to be ready (up to 5 seconds)
    Serial.print("Waiting for USB HID to be ready...");
    int attempts = 0;
    while (!CustomHID::hid.ready() && attempts < 50) {
      delay(100);
      Serial.print(".");
      attempts++;
    }
    Serial.println();
    
    if (CustomHID::hid.ready()) {
      usbStarted = true;
      Serial.println("USB HID initialized from saved descriptor and ready!");
    } else {
      Serial.println("WARNING: USB HID timeout - may not be ready");
      usbStarted = true; // Try anyway
    }
  } else {
    Serial.println("No saved descriptor - USB will start after BLE pairing");
    Serial.println("Connect a BLE remote and use the web UI to save its descriptor");
  }
  // ==========================================================

  // Start Wi-Fi AP with saved or default settings
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid.c_str(), apPass.c_str());
  Serial.print("AP SSID: "); Serial.println(apSsid);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // Start mDNS responder
  if (MDNS.begin(mdnsHost.c_str())) {
    Serial.print("mDNS hostname: http://");
    Serial.print(mdnsHost);
    Serial.println(".local");
    MDNS.addService("http", "tcp", 80);
  } else {
    Serial.println("mDNS failed to start");
  }

  // Init BLE (NimBLE) - MUST be before checking bonds
  NimBLEDevice::init("HTPC-BLEBoot");
  
  // Configure security for pairing (from working code)
  NimBLEDevice::setSecurityAuth(true, true, true); // bonding, MITM, SC
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // Just Works pairing
  
  // NOW we can check bonds (after BLE init)
  int numBonds = NimBLEDevice::getNumBonds();
  
  // If no saved device but bonds exist, clear bonds (happens after forget)
  if (savedAddr.length() == 0 && numBonds > 0) {
    Serial.printf("No saved device but found %d bond(s) - clearing bonds\n", numBonds);
    NimBLEDevice::deleteAllBonds();
    delay(200);
  } else if (savedAddr.length() > 0 && numBonds > 0) {
    Serial.println("Detected saved device with bond - enabling fast initial scan");
    firstBootAfterPairing = true;
  }
  
  pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);

  // HTTP routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ r->send_P(200, "text/html", INDEX_HTML); });
  server.on("/mapping", HTTP_GET, [](AsyncWebServerRequest* r){ r->send_P(200, "text/html", MAPPING_HTML); });
  
  // Advanced settings endpoints (MUST come before /advanced page route)
  server.on("/advanced/wifi", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("ssid")) {
      r->send(400, "application/json", "{\"ok\":false,\"err\":\"missing ssid\"}");
      return;
    }
    
    String ssid = r->getParam("ssid")->value();
    String pass = r->hasParam("pass") ? r->getParam("pass")->value() : String("");
    String mdns = r->hasParam("mdns") ? r->getParam("mdns")->value() : String("remote");
    
    // Save to preferences
    prefs.begin("cfg", false);
    prefs.putString("apSsid", ssid);
    prefs.putString("apPass", pass);
    prefs.putString("mdnsHost", mdns);
    prefs.end();
    
    r->send(200, "application/json", "{\"ok\":true}");
    
    delay(100);
    ESP.restart();
  });

  server.on("/advanced/reboot", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send(200, "application/json", "{\"ok\":true}");
    delay(100);
    ESP.restart();
  });

  server.on("/advanced/factory", HTTP_GET, [](AsyncWebServerRequest* r){
    Serial.println("Factory reset - clearing all preferences...");
    
    // Clear all preferences
    prefs.begin("cfg", false);
    prefs.clear();
    prefs.end();
    
    prefs.begin("maps", false);
    prefs.clear();
    prefs.end();
    
    r->send(200, "application/json", "{\"ok\":true}");
    
    delay(100);
    ESP.restart();
  });

  server.on("/advanced/delay", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("ms")) {
      r->send(400, "application/json", "{\"error\":\"Missing ms parameter\"}");
      return;
    }
    int ms = r->getParam("ms")->value().toInt();
    if (ms < 2 || ms > 200) {
      r->send(400, "application/json", "{\"error\":\"Value must be between 2 and 200\"}");
      return;
    }
    inputDelayMs = (uint8_t)ms;
    
    // Save to NVS
    prefs.begin("cfg", false);
    prefs.putUChar("inputDelay", inputDelayMs);
    prefs.end();
    
    Serial.printf("Input delay set to %dms\n", inputDelayMs);
    r->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/advanced/connled", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("enabled")) {
      r->send(400, "application/json", "{\"error\":\"Missing enabled parameter\"}");
      return;
    }
    int enabled = r->getParam("enabled")->value().toInt();
    connectionLedEnabled = (enabled != 0);
    
    // Save to NVS
    prefs.begin("cfg", false);
    prefs.putBool("connLed", connectionLedEnabled);
    prefs.end();
    
    Serial.printf("Connection LED animation %s\n", connectionLedEnabled ? "enabled" : "disabled");
    r->send(200, "application/json", "{\"ok\":true}");
  });

  // Backup all settings (WiFi, maps, input delay) - exclude BLE pairing
  server.on("/advanced/backup", HTTP_GET, [](AsyncWebServerRequest* r){
    String json = "{\"ok\":true,\"backup\":{";
    json += "\"version\":\"1.01\",";
    
    // Config settings
    json += "\"config\":{";
    prefs.begin("cfg", true);
    String apSsid = prefs.getString("apSsid", "HTPC-BLEBoot");
    String apPass = prefs.getString("apPass", "");
    String mdnsHost = prefs.getString("mdnsHost", "remote");
    uint8_t delay = prefs.getUChar("inputDelay", 50);
    bool connLed = prefs.getBool("connLed", true);
    prefs.end();
    
    json += "\"apSsid\":\"" + apSsid + "\",";
    json += "\"apPass\":\"" + apPass + "\",";
    json += "\"mdnsHost\":\"" + mdnsHost + "\",";
    json += "\"inputDelay\":" + String(delay) + ",";
    json += "\"connectionLed\":" + String(connLed?"true":"false");
    json += "},";
    
    // Button maps
    json += "\"maps\":[";
    bool firstMap = true;
    for(int i=0; i<64; i++){
      if(!buttonMaps[i].active) continue; // Skip empty slots
      
      if(!firstMap) json += ",";
      firstMap = false;
      
      json += "{";
      json += "\"index\":" + String(i) + ",";
      json += "\"active\":true,";
      json += "\"label\":\"" + String(buttonMaps[i].label) + "\",";
      json += "\"reportId\":" + String(buttonMaps[i].reportId) + ",";
      json += "\"payloadLen\":" + String(buttonMaps[i].payloadLen) + ",";
      json += "\"payload\":\"";
      for(int j=0; j<8; j++){
        char hex[3];
        sprintf(hex, "%02X", buttonMaps[i].payload[j]);
        json += String(hex);
      }
      json += "\",";
      json += "\"action\":" + String(buttonMaps[i].action) + ",";
      json += "\"gpioPin\":" + String(buttonMaps[i].gpioPin) + ",";
      json += "\"gpioMode\":" + String(buttonMaps[i].gpioMode) + ",";
      json += "\"gpioTtlBehavior\":" + String(buttonMaps[i].gpioTtlBehavior) + ",";
      json += "\"ttlMs\":" + String(buttonMaps[i].ttlMs) + ",";
      json += "\"passToUsb\":" + String(buttonMaps[i].passToUsb ? "true" : "false") + ",";
      json += "\"ledR\":" + String(buttonMaps[i].ledR) + ",";
      json += "\"ledG\":" + String(buttonMaps[i].ledG) + ",";
      json += "\"ledB\":" + String(buttonMaps[i].ledB) + ",";
      json += "\"useHoldTime\":" + String(buttonMaps[i].useHoldTime ? "true" : "false") + ",";
      json += "\"holdTimeSec\":" + String(buttonMaps[i].holdTimeSec) + ",";
      json += "\"applyToUsb\":" + String(buttonMaps[i].applyToUsb ? "true" : "false");
      json += "}";
    }
    json += "]";
    json += "}}";
    
    r->send(200, "application/json", json);
  });

  // Restore settings from backup
  server.on("/advanced/restore", HTTP_POST, [](AsyncWebServerRequest* r){}, NULL,
    [](AsyncWebServerRequest* r, uint8_t *data, size_t len, size_t index, size_t total){
      if(index == 0){
        Serial.println("Starting backup restore...");
      }
      
      // Parse JSON (simplified - assumes data arrives in one chunk)
      if(index + len == total){
        String jsonStr = String((char*)data).substring(0, len);
        
        // Extract config values
        int ssidStart = jsonStr.indexOf("\"apSsid\":\"") + 10;
        int ssidEnd = jsonStr.indexOf("\"", ssidStart);
        String apSsid = jsonStr.substring(ssidStart, ssidEnd);
        
        int passStart = jsonStr.indexOf("\"apPass\":\"") + 10;
        int passEnd = jsonStr.indexOf("\"", passStart);
        String apPass = jsonStr.substring(passStart, passEnd);
        
        int mdnsStart = jsonStr.indexOf("\"mdnsHost\":\"") + 12;
        int mdnsEnd = jsonStr.indexOf("\"", mdnsStart);
        String mdnsHost = (mdnsStart > 12 && mdnsEnd > mdnsStart) ? jsonStr.substring(mdnsStart, mdnsEnd) : String("remote");
        
        int delayStart = jsonStr.indexOf("\"inputDelay\":") + 13;
        int delayEnd = jsonStr.indexOf("}", delayStart);
        if(delayEnd == -1 || delayEnd > delayStart + 10) delayEnd = jsonStr.indexOf(",", delayStart);
        uint8_t inputDelay = jsonStr.substring(delayStart, delayEnd).toInt();
        
        int connLedStart = jsonStr.indexOf("\"connectionLed\":") + 16;
        bool connLed = true; // Default to true
        if(connLedStart > 16) {
          int connLedEnd = jsonStr.indexOf("}", connLedStart);
          if(connLedEnd == -1 || connLedEnd > connLedStart + 10) connLedEnd = jsonStr.indexOf(",", connLedStart);
          String connLedStr = jsonStr.substring(connLedStart, connLedEnd);
          connLed = (connLedStr == "true");
        }
        
        // Save config
        prefs.begin("cfg", false);
        prefs.putString("apSsid", apSsid);
        prefs.putString("apPass", apPass);
        prefs.putString("mdnsHost", mdnsHost);
        prefs.putUChar("inputDelay", inputDelay);
        prefs.putBool("connLed", connLed);
        prefs.end();
        
        inputDelayMs = inputDelay;
        connectionLedEnabled = connLed;
        
        Serial.printf("Config restored: SSID=%s, mDNS=%s, Delay=%dms, LED=%s\n", apSsid.c_str(), mdnsHost.c_str(), inputDelay, connLed?"on":"off");
        
        // Clear existing maps
        for(int i=0; i<64; i++){
          buttonMaps[i] = ButtonMap();
        }
        
        // Parse and restore maps
        int mapsStart = jsonStr.indexOf("\"maps\":[") + 8;
        int mapsEnd = jsonStr.indexOf("]", mapsStart);
        String mapsJson = jsonStr.substring(mapsStart, mapsEnd);
        
        int mapCount = 0;
        int pos = 0;
        
        prefs.begin("maps", false);
        char key[16];
        
        while(pos < mapsJson.length()){
          int objStart = mapsJson.indexOf("{", pos);
          if(objStart == -1) break;
          int objEnd = mapsJson.indexOf("}", objStart);
          if(objEnd == -1) break;
          
          String mapJson = mapsJson.substring(objStart + 1, objEnd);
          
          // Parse map fields
          int mapIndex = getValue(mapJson, "index").toInt();
          if(mapIndex < 0 || mapIndex >= 64){
            pos = objEnd + 1;
            continue;
          }
          
          ButtonMap* map = &buttonMaps[mapIndex];
          map->active = true;
          String label = getValue(mapJson, "label");
          label.toCharArray(map->label, sizeof(map->label));
          map->reportId = getValue(mapJson, "reportId").toInt();
          map->payloadLen = getValue(mapJson, "payloadLen").toInt();
          
          // Parse hex payload
          String payloadHex = getValue(mapJson, "payload");
          for(int j=0; j<8 && j*2+1<payloadHex.length(); j++){
            String hexByte = payloadHex.substring(j*2, j*2+2);
            map->payload[j] = strtol(hexByte.c_str(), NULL, 16);
          }
          
          map->action = getValue(mapJson, "action").toInt();
          map->gpioPin = getValue(mapJson, "gpioPin").toInt();
          map->gpioMode = getValue(mapJson, "gpioMode").toInt();
          map->gpioTtlBehavior = getValue(mapJson, "gpioTtlBehavior").toInt();
          map->ttlMs = getValue(mapJson, "ttlMs").toInt();
          map->passToUsb = getValue(mapJson, "passToUsb") == "true";
          map->ledR = getValue(mapJson, "ledR").toInt();
          map->ledG = getValue(mapJson, "ledG").toInt();
          map->ledB = getValue(mapJson, "ledB").toInt();
          map->useHoldTime = getValue(mapJson, "useHoldTime") == "true";
          map->holdTimeSec = getValue(mapJson, "holdTimeSec").toInt();
          map->applyToUsb = getValue(mapJson, "applyToUsb") == "true";
          
          // Save to NVS
          snprintf(key, sizeof(key), "active%d", mapIndex);
          prefs.putBool(key, map->active);
          snprintf(key, sizeof(key), "label%d", mapIndex);
          prefs.putString(key, map->label);
          snprintf(key, sizeof(key), "rid%d", mapIndex);
          prefs.putUChar(key, map->reportId);
          snprintf(key, sizeof(key), "plen%d", mapIndex);
          prefs.putUChar(key, map->payloadLen);
          snprintf(key, sizeof(key), "pay%d", mapIndex);
          prefs.putBytes(key, map->payload, 8);
          snprintf(key, sizeof(key), "act%d", mapIndex);
          prefs.putUChar(key, map->action);
          snprintf(key, sizeof(key), "gpio%d", mapIndex);
          prefs.putUChar(key, map->gpioPin);
          snprintf(key, sizeof(key), "gpioMode%d", mapIndex);
          prefs.putUChar(key, map->gpioMode);
          snprintf(key, sizeof(key), "gpioTtlBeh%d", mapIndex);
          prefs.putUChar(key, map->gpioTtlBehavior);
          snprintf(key, sizeof(key), "ttl%d", mapIndex);
          prefs.putUShort(key, map->ttlMs);
          snprintf(key, sizeof(key), "pass%d", mapIndex);
          prefs.putBool(key, map->passToUsb);
          snprintf(key, sizeof(key), "ledR%d", mapIndex);
          prefs.putUChar(key, map->ledR);
          snprintf(key, sizeof(key), "ledG%d", mapIndex);
          prefs.putUChar(key, map->ledG);
          snprintf(key, sizeof(key), "ledB%d", mapIndex);
          prefs.putUChar(key, map->ledB);
          snprintf(key, sizeof(key), "useHoldTime%d", mapIndex);
          prefs.putBool(key, map->useHoldTime);
          snprintf(key, sizeof(key), "holdTime%d", mapIndex);
          prefs.putUChar(key, map->holdTimeSec);
          snprintf(key, sizeof(key), "applyUsb%d", mapIndex);
          prefs.putBool(key, map->applyToUsb);
          
          mapCount++;
          pos = objEnd + 1;
        }
        
        prefs.end();
        
        Serial.printf("Restored %d button maps\n", mapCount);
        
        r->send(200, "application/json", "{\"ok\":true,\"restored\":" + String(mapCount) + "}");
        
        // Reboot after restore
        delay(500);
        ESP.restart();
      }
    }
  );
  
  server.on("/advanced", HTTP_GET, [](AsyncWebServerRequest* r){ r->send_P(200, "text/html", ADVANCED_HTML); });
  server.on("/status", HTTP_GET, handleStatus);

  // Start a background BLE scan (non-blocking)
  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!pScan) { r->send(500, "application/json", "{\"started\":false}"); return; }
    if (scanRunning) { 
      Serial.println("Scan already in progress - restarting ESP32...");
      r->send(200, "application/json", "{\"restarting\":true}");
      delay(100);
      ESP.restart();
      return; 
    }
    // Can't scan while connected - disconnect first
    if (isConnected || isConnecting || connectRunning) {
      r->send(200, "application/json", "{\"started\":false,\"err\":\"disconnect first\"}");
      return;
    }
    
    // Launch scan task
    scanRunning = true;
    if (xSemaphoreTake(discoveredMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      discovered.clear();
      xSemaphoreGive(discoveredMutex);
    }
    
    xTaskCreatePinnedToCore(
      [](void* p){
        // Run blocking scan here to avoid blocking async web task
        pScan->clearResults();
        NimBLEScanResults res = pScan->start(5, false);
        std::vector<DiscoveredDev> tmp;
        int n = res.getCount(); tmp.reserve(n);
        for (int i=0;i<n;i++) {
          NimBLEAdvertisedDevice dev = res.getDevice(i);
          // Only include devices with names
          if (dev.getName().empty()) continue;
          DiscoveredDev d; d.addr = String(dev.getAddress().toString().c_str());
          d.name = String(dev.getName().c_str());
          d.name.replace("\"","'");
          d.rssi = dev.getRSSI();
          tmp.push_back(d);
        }
        pScan->clearResults();
        
        // Safely update discovered vector
        if (xSemaphoreTake(discoveredMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          discovered.swap(tmp);
          xSemaphoreGive(discoveredMutex);
        }
        
        scanRunning = false;
        vTaskDelete(nullptr);
      },
      "scanTask", 4096, nullptr, 1, &scanTaskHandle, 1
    );
    r->send(200, "application/json", "{\"started\":true}");
  });

  // Poll scan results (non-blocking)
  server.on("/scan_results", HTTP_GET, [](AsyncWebServerRequest* r){
    String json = "{";
    json += "\"running\":"; json += scanRunning?"true":"false"; json += ",\"devices\":[";
    
    if (xSemaphoreTake(discoveredMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (size_t i=0;i<discovered.size();++i) {
        if (i) json += ",";
        json += "{\"address\":\"" + discovered[i].addr + "\",\"name\":\"" + discovered[i].name + "\",\"rssi\":" + String(discovered[i].rssi) + "}";
      }
      xSemaphoreGive(discoveredMutex);
    }
    
    json += "]}";
    r->send(200, "application/json", json);
  });

  server.on("/connect", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("addr")) { r->send(400, "application/json", "{\"ok\":false,\"err\":\"missing addr\"}"); return; }
    if (connectRunning || isConnecting) { r->send(200, "application/json", "{\"ok\":false,\"err\":\"already connecting\"}"); return; }
    
    String addr = r->getParam("addr")->value();
    String name = r->hasParam("name") ? r->getParam("name")->value() : String("");
    
    // If already connected, disconnect first to allow reconnection
    if (isConnected || (pClient && pClient->isConnected())) {
      Serial.println("Already connected - disconnecting to allow reconnection");
      if (connectTaskHandle) { 
        vTaskDelete(connectTaskHandle); 
        connectTaskHandle = nullptr; 
        connectRunning = false;
      }
      if (pClient && pClient->isConnected()) pClient->disconnect();
      isConnected=false; isConnecting=false; activeAddr=""; activeName="";
      delay(500); // Give time for disconnect to complete
    }
    
    startConnectTask(addr, name);
    r->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/disconnect", HTTP_GET, [](AsyncWebServerRequest* r){
    // Cancel any pending connect task
    if (connectTaskHandle) { 
      vTaskDelete(connectTaskHandle); 
      connectTaskHandle = nullptr; 
      connectRunning = false;
    }
    if (pClient && pClient->isConnected()) pClient->disconnect();
    isConnected=false; isConnecting=false; activeAddr=""; activeName="";
    r->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/forget", HTTP_GET, [](AsyncWebServerRequest* r){
    Serial.println("Forget device - clearing preferences and rebooting...");
    
    // Clear saved device preferences
    prefs.begin("cfg", false);
    prefs.putString("remoteAddr", "");
    prefs.putString("remoteName", "");
    prefs.end();
    
    // Send response before rebooting
    r->send(200, "application/json", "{\"ok\":true}");
    
    // Reboot - bonds will be cleared on startup
    delay(100);
    ESP.restart();
  });

  server.on("/learn", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!isConnected) {
      r->send(200, "application/json", "{\"ok\":false,\"err\":\"not connected\"}");
      return;
    }
    if (!r->hasParam("idx")) {
      r->send(400, "application/json", "{\"ok\":false,\"err\":\"missing idx\"}");
      return;
    }
    int idx = r->getParam("idx")->value().toInt();
    if (idx < 0 || idx >= MAX_BUTTON_MAPS || !buttonMaps[idx].active) {
      r->send(400, "application/json", "{\"ok\":false,\"err\":\"invalid idx\"}");
      return;
    }
    Serial.printf("Starting learn mode for button map #%d...\n", idx);
    learnMode = true;
    learnMapIndex = idx;
    learnModeStartTime = millis();
    r->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/map/add", HTTP_GET, [](AsyncWebServerRequest* r){
    // Find first available slot
    int idx = -1;
    for (int i = 0; i < MAX_BUTTON_MAPS; i++) {
      if (!buttonMaps[i].active) {
        idx = i;
        break;
      }
    }
    if (idx == -1) {
      r->send(200, "application/json", "{\"ok\":false,\"err\":\"max maps reached\"}");
      return;
    }
    
    // Initialize new map with defaults
    buttonMaps[idx].active = true;
    snprintf(buttonMaps[idx].label, sizeof(buttonMaps[idx].label), "Button %d", idx + 1);
    buttonMaps[idx].reportId = 3;
    buttonMaps[idx].payloadLen = 1;
    memset(buttonMaps[idx].payload, 0, sizeof(buttonMaps[idx].payload));
    buttonMaps[idx].payload[0] = 0x01;
    buttonMaps[idx].action = 2; // LED by default
    buttonMaps[idx].gpioPin = 2;
    buttonMaps[idx].passToUsb = false;
    buttonMaps[idx].ledR = 0;
    buttonMaps[idx].ledG = 255;
    buttonMaps[idx].ledB = 0;
    activeButtonMaps++;
    
    // Save to preferences
    prefs.begin("maps", false);
    char key[16];
    snprintf(key, sizeof(key), "active%d", idx);
    prefs.putBool(key, true);
    snprintf(key, sizeof(key), "label%d", idx);
    prefs.putString(key, buttonMaps[idx].label);
    snprintf(key, sizeof(key), "rid%d", idx);
    prefs.putUChar(key, buttonMaps[idx].reportId);
    snprintf(key, sizeof(key), "plen%d", idx);
    prefs.putUChar(key, buttonMaps[idx].payloadLen);
    snprintf(key, sizeof(key), "pay%d", idx);
    prefs.putBytes(key, buttonMaps[idx].payload, 8);
    snprintf(key, sizeof(key), "act%d", idx);
    prefs.putUChar(key, buttonMaps[idx].action);
    snprintf(key, sizeof(key), "gpio%d", idx);
    prefs.putUChar(key, buttonMaps[idx].gpioPin);
    snprintf(key, sizeof(key), "pass%d", idx);
    prefs.putBool(key, buttonMaps[idx].passToUsb);
    snprintf(key, sizeof(key), "ledR%d", idx);
    prefs.putUChar(key, buttonMaps[idx].ledR);
    snprintf(key, sizeof(key), "ledG%d", idx);
    prefs.putUChar(key, buttonMaps[idx].ledG);
    snprintf(key, sizeof(key), "ledB%d", idx);
    prefs.putUChar(key, buttonMaps[idx].ledB);
    prefs.end();
    
    Serial.printf("Added button map #%d\n", idx);
    r->send(200, "application/json", "{\"ok\":true,\"idx\":" + String(idx) + "}");
  });

  server.on("/map/update", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("idx")) {
      r->send(400, "application/json", "{\"ok\":false,\"err\":\"missing idx\"}");
      return;
    }
    int idx = r->getParam("idx")->value().toInt();
    if (idx < 0 || idx >= MAX_BUTTON_MAPS || !buttonMaps[idx].active) {
      r->send(400, "application/json", "{\"ok\":false,\"err\":\"invalid idx\"}");
      return;
    }
    
    bool changed = false;
    
    if (r->hasParam("label")) {
      String label = r->getParam("label")->value();
      strncpy(buttonMaps[idx].label, label.c_str(), sizeof(buttonMaps[idx].label) - 1);
      buttonMaps[idx].label[sizeof(buttonMaps[idx].label) - 1] = '\0';
      changed = true;
    }
    
    if (r->hasParam("action")) {
      int action = r->getParam("action")->value().toInt();
      if (action >= 0 && action <= 5) {
        buttonMaps[idx].action = action;
        changed = true;
      }
    }
    
    if (r->hasParam("gpio")) {
      uint8_t pin = r->getParam("gpio")->value().toInt();
      if (pin < 49) {
        buttonMaps[idx].gpioPin = pin;
        if (buttonMaps[idx].action == 1) {
          pinMode(pin, OUTPUT);
          digitalWrite(pin, LOW);
        }
        changed = true;
      }
    }
    
    if (r->hasParam("gpioMode")) {
      uint8_t mode = r->getParam("gpioMode")->value().toInt();
      if (mode <= 2) {
        buttonMaps[idx].gpioMode = mode;
        changed = true;
      }
    }
    
    if (r->hasParam("gpioTtlBehavior")) {
      uint8_t behavior = r->getParam("gpioTtlBehavior")->value().toInt();
      if (behavior <= 1) {
        buttonMaps[idx].gpioTtlBehavior = behavior;
        Serial.printf("Map #%d: GPIO TTL behavior set to %s\n", idx, behavior == 0 ? "OPPOSITE" : "FLOAT");
        changed = true;
      }
    }
    
    if (r->hasParam("ttl")) {
      uint16_t ttl = r->getParam("ttl")->value().toInt();
      if (ttl <= 9999) {
        buttonMaps[idx].ttlMs = ttl;
        Serial.printf("Map #%d: TTL set to %dms\n", idx, ttl);
        changed = true;
      }
    }
    
    if (r->hasParam("useHold")) {
      buttonMaps[idx].useHoldTime = (r->getParam("useHold")->value() != "0");
      changed = true;
    }
    
    if (r->hasParam("holdSec")) {
      uint8_t holdSec = r->getParam("holdSec")->value().toInt();
      if (holdSec >= 1 && holdSec <= 10) {
        buttonMaps[idx].holdTimeSec = holdSec;
        changed = true;
      }
    }
    
    if (r->hasParam("applyUsb")) {
      buttonMaps[idx].applyToUsb = (r->getParam("applyUsb")->value() != "0");
      changed = true;
    }
    
    if (r->hasParam("pass")) {
      buttonMaps[idx].passToUsb = (r->getParam("pass")->value() != "0");
      changed = true;
    }
    
    if (r->hasParam("color")) {
      // Parse hex color #RRGGBB
      String color = r->getParam("color")->value();
      if (color.length() >= 7 && color[0] == '#') {
        long hexVal = strtol(color.c_str() + 1, NULL, 16);
        buttonMaps[idx].ledR = (hexVal >> 16) & 0xFF;
        buttonMaps[idx].ledG = (hexVal >> 8) & 0xFF;
        buttonMaps[idx].ledB = hexVal & 0xFF;
        changed = true;
      }
    }
    
    if (r->hasParam("keyCode")) {
      String keyCodeStr = r->getParam("keyCode")->value();
      buttonMaps[idx].keyCode = (uint16_t)strtol(keyCodeStr.c_str(), NULL, 16);
      changed = true;
    }
    
    if (r->hasParam("keyMod")) {
      buttonMaps[idx].keyModifier = (uint8_t)r->getParam("keyMod")->value().toInt();
      changed = true;
    }
    
    if (r->hasParam("mediaCode")) {
      String mediaCodeStr = r->getParam("mediaCode")->value();
      buttonMaps[idx].mediaCode = (uint16_t)strtol(mediaCodeStr.c_str(), NULL, 16);
      changed = true;
    }
    
    if (changed) {
      // Save to preferences
      prefs.begin("maps", false);
      char key[16];
      snprintf(key, sizeof(key), "label%d", idx);
      prefs.putString(key, buttonMaps[idx].label);
      snprintf(key, sizeof(key), "act%d", idx);
      prefs.putUChar(key, buttonMaps[idx].action);
      snprintf(key, sizeof(key), "gpio%d", idx);
      prefs.putUChar(key, buttonMaps[idx].gpioPin);
      snprintf(key, sizeof(key), "gpioMode%d", idx);
      prefs.putUChar(key, buttonMaps[idx].gpioMode);
      snprintf(key, sizeof(key), "gpioTtlBeh%d", idx);
      prefs.putUChar(key, buttonMaps[idx].gpioTtlBehavior);
      snprintf(key, sizeof(key), "ttl%d", idx);
      prefs.putUShort(key, buttonMaps[idx].ttlMs);
      snprintf(key, sizeof(key), "useHold%d", idx);
      prefs.putBool(key, buttonMaps[idx].useHoldTime);
      snprintf(key, sizeof(key), "holdSec%d", idx);
      prefs.putUChar(key, buttonMaps[idx].holdTimeSec);
      snprintf(key, sizeof(key), "applyUsb%d", idx);
      prefs.putBool(key, buttonMaps[idx].applyToUsb);
      snprintf(key, sizeof(key), "pass%d", idx);
      prefs.putBool(key, buttonMaps[idx].passToUsb);
      snprintf(key, sizeof(key), "ledR%d", idx);
      prefs.putUChar(key, buttonMaps[idx].ledR);
      snprintf(key, sizeof(key), "ledG%d", idx);
      prefs.putUChar(key, buttonMaps[idx].ledG);
      snprintf(key, sizeof(key), "ledB%d", idx);
      prefs.putUChar(key, buttonMaps[idx].ledB);
      snprintf(key, sizeof(key), "keyCode%d", idx);
      prefs.putUShort(key, buttonMaps[idx].keyCode);
      snprintf(key, sizeof(key), "keyMod%d", idx);
      prefs.putUChar(key, buttonMaps[idx].keyModifier);
      snprintf(key, sizeof(key), "mediaCode%d", idx);
      prefs.putUShort(key, buttonMaps[idx].mediaCode);
      prefs.end();
    }
    
    r->send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/map/delete", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("idx")) {
      r->send(400, "application/json", "{\"ok\":false,\"err\":\"missing idx\"}");
      return;
    }
    int idx = r->getParam("idx")->value().toInt();
    if (idx < 0 || idx >= MAX_BUTTON_MAPS || !buttonMaps[idx].active) {
      r->send(400, "application/json", "{\"ok\":false,\"err\":\"invalid idx\"}");
      return;
    }
    
    // Mark as inactive
    buttonMaps[idx].active = false;
    activeButtonMaps--;
    
    // Clear from preferences
    prefs.begin("maps", false);
    char key[16];
    snprintf(key, sizeof(key), "active%d", idx);
    prefs.putBool(key, false);
    prefs.end();
    
    Serial.printf("Deleted button map #%d\n", idx);
    r->send(200, "application/json", "{\"ok\":true}");
  });

  // ===== HID Descriptor Management Endpoints =====
  
  // Save currently connected BLE device's descriptor
  server.on("/descriptor/save", HTTP_GET, [](AsyncWebServerRequest* r){
    if (capturedDescriptorLen == 0) {
      r->send(400, "application/json", "{\"ok\":false,\"error\":\"No descriptor captured. Connect a BLE remote first.\"}");
      return;
    }
    
    if (saveDescriptor(capturedDescriptor, capturedDescriptorLen)) {
      String json = "{\"ok\":true,\"length\":" + String(capturedDescriptorLen) + ",\"message\":\"Descriptor saved. Reboot to apply.\"}";
      r->send(200, "application/json", json);
    } else {
      r->send(500, "application/json", "{\"ok\":false,\"error\":\"Failed to save descriptor to NVS\"}");
    }
  });
  
  // Delete saved descriptor
  server.on("/descriptor/delete", HTTP_GET, [](AsyncWebServerRequest* r){
    if (deleteSavedDescriptor()) {
      r->send(200, "application/json", "{\"ok\":true,\"message\":\"Descriptor deleted. Reboot to apply.\"}");
    } else {
      r->send(400, "application/json", "{\"ok\":false,\"error\":\"No saved descriptor to delete\"}");
    }
  });
  
  // Get descriptor status
  server.on("/descriptor/status", HTTP_GET, [](AsyncWebServerRequest* r){
    String json = "{\"ok\":true,";
    json += "\"hasSaved\":" + String(hasSavedDescriptor() ? "true" : "false") + ",";
    json += "\"savedLength\":" + String(hasSavedDescriptor() ? "169" : "0") + ",";
    json += "\"currentLength\":" + String(capturedDescriptorLen) + ",";
    json += "\"usbStarted\":" + String(usbStarted ? "true" : "false");
    json += "}";
    r->send(200, "application/json", json);
  });
  
  // Download descriptor as text file
  server.on("/descriptor/download", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!hasSavedDescriptor()) {
      r->send(404, "text/plain", "No saved descriptor found");
      return;
    }
    
    // Load descriptor from NVS
    uint8_t desc[256];
    size_t len = 0;
    prefs.begin("hid_desc", true);
    len = prefs.getBytesLength("desc");
    if (len > 0 && len <= 256) {
      prefs.getBytes("desc", desc, len);
    }
    prefs.end();
    
    if (len == 0) {
      r->send(404, "text/plain", "Failed to load descriptor");
      return;
    }
    
    // Convert to hex string format (2 hex chars per byte, space separated)
    String hexStr = "";
    for (size_t i = 0; i < len; i++) {
      char buf[4];
      sprintf(buf, "%02X", desc[i]);
      hexStr += buf;
      if (i < len - 1) hexStr += " ";
    }
    
    // Send as downloadable text file
    AsyncWebServerResponse *response = r->beginResponse(200, "text/plain", hexStr);
    response->addHeader("Content-Disposition", "attachment; filename=\"hid_descriptor.txt\"");
    r->send(response);
    Serial.println("Descriptor downloaded by user");
  });
  
  // Upload custom descriptor
  server.on("/descriptor/upload", HTTP_POST, [](AsyncWebServerRequest* r){
    r->send(200, "application/json", "{\"ok\":true,\"message\":\"Upload complete. Reboot to apply.\"}");
  }, NULL, [](AsyncWebServerRequest* r, uint8_t *data, size_t len, size_t index, size_t total){
    // Body handler for receiving descriptor data
    static uint8_t uploadBuffer[256];
    static size_t uploadLen = 0;
    
    if (index == 0) {
      // First chunk - reset buffer
      uploadLen = 0;
      Serial.printf("Starting descriptor upload, total size: %d bytes\n", total);
    }
    
    // Parse hex string format (space or comma separated hex bytes)
    String dataStr = "";
    for (size_t i = 0; i < len; i++) {
      dataStr += (char)data[i];
    }
    
    // Remove whitespace and parse hex bytes
    dataStr.trim();
    dataStr.replace(",", " ");
    dataStr.replace("  ", " ");
    
    int startIdx = 0;
    while (startIdx < dataStr.length() && uploadLen < 256) {
      int spaceIdx = dataStr.indexOf(' ', startIdx);
      if (spaceIdx == -1) spaceIdx = dataStr.length();
      
      String hexByte = dataStr.substring(startIdx, spaceIdx);
      hexByte.trim();
      
      if (hexByte.length() > 0) {
        uploadBuffer[uploadLen++] = (uint8_t)strtol(hexByte.c_str(), NULL, 16);
      }
      
      startIdx = spaceIdx + 1;
    }
    
    // If this is the last chunk, save to NVS
    if (index + len >= total) {
      if (uploadLen > 0 && uploadLen <= 256) {
        if (saveDescriptor(uploadBuffer, uploadLen)) {
          Serial.printf("Custom descriptor uploaded and saved (%d bytes)\n", uploadLen);
        } else {
          Serial.println("Failed to save uploaded descriptor");
        }
      } else {
        Serial.printf("Invalid descriptor length: %d bytes\n", uploadLen);
      }
    }
  });

  server.begin();
}

void loop() {
  delay(100);
  
  // Update no-device pulsing LED (only if no device saved)
  updateNoPairPulse();
  
  // Periodic battery reading (every 60 seconds when connected)
  if (isConnected && pClient && pClient->isConnected()) {
    unsigned long now = millis();
    if (now - lastBatteryRead >= BATTERY_READ_INTERVAL) {
      lastBatteryRead = now;
      readBatteryLevel(pClient);
    }
  }
  
  // Check hold time requirements for buttons that are still pressed
  for (int i = 0; i < MAX_BUTTON_MAPS; i++) {
    if (buttonPressStates[i].isPressed && !buttonPressStates[i].actionTriggered && buttonMaps[i].active && buttonMaps[i].useHoldTime) {
      unsigned long holdDuration = millis() - buttonPressStates[i].pressStartTime;
      unsigned long requiredHoldMs = buttonMaps[i].holdTimeSec * 1000UL;
      
      // Check if hold time has been met
      if (holdDuration >= requiredHoldMs) {
        buttonPressStates[i].actionTriggered = true;
        Serial.printf("Button '%s' held for %lu sec - executing action\n", buttonMaps[i].label, buttonMaps[i].holdTimeSec);
        
        // Execute the action
        if (buttonMaps[i].action == 0) {
          // Forward to USB
          uint16_t consumerCode = buttonMaps[i].payload[0] | (buttonMaps[i].payload[1] << 8);
          if (buttonMaps[i].applyToUsb) {
            // Single send mode - send once
            if (!usbSentOnce[i]) {
              usbSendConsumerMomentary(consumerCode);
              usbSentOnce[i] = true;
              Serial.printf("  -> Forwarded original button (0x%04X) to USB [SINGLE]\n", consumerCode);
            }
          } else {
            // Repeat mode - send continuously (will repeat in subsequent loops)
            usbSendConsumerMomentary(consumerCode);
          }
        } else if (buttonMaps[i].action == 2) {
          // LED custom color
          pixel.setPixelColor(0, pixel.Color(buttonMaps[i].ledR, buttonMaps[i].ledG, buttonMaps[i].ledB));
          Serial.printf("LED -> RGB(%d,%d,%d)", buttonMaps[i].ledR, buttonMaps[i].ledG, buttonMaps[i].ledB);
          pixel.show();
          
          // Start TTL timer if configured
          if (buttonMaps[i].ttlMs > 0) {
            ttlTimers[i].active = true;
            ttlTimers[i].startTime = millis();
            ttlTimers[i].mapIndex = i;
            Serial.printf(" (TTL: %dms)\n", buttonMaps[i].ttlMs);
          } else {
            Serial.println();
          }
          
          // Handle USB forwarding
          if (buttonMaps[i].passToUsb) {
            uint16_t consumerCode = buttonMaps[i].payload[0] | (buttonMaps[i].payload[1] << 8);
            if (buttonMaps[i].applyToUsb) {
              // Single send
              if (!usbSentOnce[i]) {
                usbSendConsumerMomentary(consumerCode);
                usbSentOnce[i] = true;
                Serial.printf("  -> Forwarded original button (0x%04X) to USB [SINGLE]\n", consumerCode);
              }
            } else {
              // Repeat mode
              usbSendConsumerMomentary(consumerCode);
            }
          }
        } else if (buttonMaps[i].action == 4) {
          // Keyboard action
          if (buttonMaps[i].applyToUsb) {
            // Single send mode - send once only (press + release for a keystroke)
            if (!usbSentOnce[i]) {
              uint8_t keys[6] = {(uint8_t)(buttonMaps[i].keyCode & 0xFF), 0, 0, 0, 0, 0};
              usbSendKeyboardReport(buttonMaps[i].keyModifier, keys);
              Serial.printf("Keyboard -> Key 0x%02X (modifier 0x%02X) [SINGLE]\n", buttonMaps[i].keyCode, buttonMaps[i].keyModifier);
              delay(50); // Brief hold
              uint8_t release[6] = {0, 0, 0, 0, 0, 0};
              usbSendKeyboardReport(0, release);
              Serial.println("Keyboard -> Released [SINGLE]");
              usbSentOnce[i] = true;
            }
          } else {
            // Repeat mode - send continuously
            uint8_t keys[6] = {(uint8_t)(buttonMaps[i].keyCode & 0xFF), 0, 0, 0, 0, 0};
            usbSendKeyboardReport(buttonMaps[i].keyModifier, keys);
            Serial.printf("Keyboard -> Key 0x%02X (modifier 0x%02X) [REPEAT]\n", buttonMaps[i].keyCode, buttonMaps[i].keyModifier);
          }
        } else if (buttonMaps[i].action == 5) {
          // Media action
          if (buttonMaps[i].applyToUsb) {
            // Single send mode - send once only (press + release)
            if (!usbSentOnce[i]) {
              usbSendConsumer(buttonMaps[i].mediaCode);
              Serial.printf("Media -> Consumer 0x%04X [SINGLE]\n", buttonMaps[i].mediaCode);
              delay(50); // Brief hold
              usbSendConsumerRelease();
              Serial.println("Media -> Released [SINGLE]");
              usbSentOnce[i] = true;
            }
          } else {
            // Repeat mode - send continuously
            usbSendConsumer(buttonMaps[i].mediaCode);
            Serial.printf("Media -> Consumer 0x%04X [REPEAT]\n", buttonMaps[i].mediaCode);
          }
        } else if (buttonMaps[i].action == 1) {
          // GPIO action - execute when hold time is met
          if (buttonMaps[i].gpioMode == 0) {
            // Toggle mode - use persistent toggle state
            gpioToggleStates[i] = !gpioToggleStates[i];
            gpioStates[i] = gpioToggleStates[i];
            digitalWrite(buttonMaps[i].gpioPin, gpioStates[i] ? HIGH : LOW);
            Serial.printf("GPIO%d TOGGLE -> %s", buttonMaps[i].gpioPin, gpioStates[i] ? "HIGH" : "LOW");
          } else if (buttonMaps[i].gpioMode == 1) {
            // Pull HIGH mode
            digitalWrite(buttonMaps[i].gpioPin, HIGH);
            gpioStates[i] = true;
            Serial.printf("GPIO%d -> HIGH", buttonMaps[i].gpioPin);
          } else if (buttonMaps[i].gpioMode == 2) {
            // Pull LOW mode
            digitalWrite(buttonMaps[i].gpioPin, LOW);
            gpioStates[i] = false;
            Serial.printf("GPIO%d -> LOW", buttonMaps[i].gpioPin);
          }
          
          // Start TTL timer if configured
          if (buttonMaps[i].ttlMs > 0) {
            ttlTimers[i].active = true;
            ttlTimers[i].startTime = millis();
            ttlTimers[i].mapIndex = i;
            Serial.printf(" (TTL: %dms)\n", buttonMaps[i].ttlMs);
          } else {
            Serial.println();
          }
          
          // Handle USB forwarding
          if (buttonMaps[i].passToUsb) {
            uint16_t consumerCode = buttonMaps[i].payload[0] | (buttonMaps[i].payload[1] << 8);
            if (buttonMaps[i].applyToUsb) {
              // Single send mode
              if (!usbSentOnce[i]) {
                usbSendConsumerMomentary(consumerCode);
                usbSentOnce[i] = true;
                Serial.printf("  -> Forwarded original button (0x%04X) to USB [SINGLE]\n", consumerCode);
              }
            } else {
              // Repeat mode
              usbSendConsumerMomentary(consumerCode);
            }
          }
        }
        // Note: GPIO action now executes when hold time is met (above)
      }
    } else if (buttonPressStates[i].isPressed && buttonPressStates[i].actionTriggered && buttonMaps[i].active && buttonMaps[i].useHoldTime) {
      // Continue sending USB data if in repeat mode (after hold time was already met)
      uint16_t consumerCode = buttonMaps[i].payload[0] | (buttonMaps[i].payload[1] << 8);
      
      if (buttonMaps[i].action == 0 && !buttonMaps[i].applyToUsb) {
        usbSendConsumerMomentary(consumerCode);
      } else if (buttonMaps[i].action == 2 && buttonMaps[i].passToUsb && !buttonMaps[i].applyToUsb) {
        usbSendConsumerMomentary(consumerCode);
      } else if (buttonMaps[i].action == 4 && !buttonMaps[i].applyToUsb) {
        // Keyboard repeat mode
        uint8_t keys[6] = {(uint8_t)(buttonMaps[i].keyCode & 0xFF), 0, 0, 0, 0, 0};
        usbSendKeyboardReport(buttonMaps[i].keyModifier, keys);
      } else if (buttonMaps[i].action == 5 && !buttonMaps[i].applyToUsb) {
        // Media repeat mode
        usbSendConsumer(buttonMaps[i].mediaCode);
      }
    }
  }
  
  // Handle keyboard/media repeat mode when hold time is enabled but applyToUsb is false
  // (Send immediately without waiting for hold time)
  // Also handle GPIO/LED USB forwarding in repeat mode
  for (int i = 0; i < MAX_BUTTON_MAPS; i++) {
    if (buttonPressStates[i].isPressed && buttonMaps[i].active && buttonMaps[i].useHoldTime && !buttonMaps[i].applyToUsb) {
      if ((buttonMaps[i].action == 4 || buttonMaps[i].action == 5) && !buttonPressStates[i].actionTriggered) {
        // Mark as triggered immediately (skip hold time requirement for keyboard/media)
        buttonPressStates[i].actionTriggered = true;
        Serial.printf("Button '%s' - immediate repeat mode (no hold time)\n", buttonMaps[i].label);
      }
      
      if ((buttonMaps[i].action == 1 || buttonMaps[i].action == 2) && buttonMaps[i].passToUsb && !buttonPressStates[i].actionTriggered) {
        // Mark GPIO/LED as triggered immediately when USB forwarding with applyToUsb=false
        buttonPressStates[i].actionTriggered = true;
        Serial.printf("Button '%s' - immediate USB forward mode (no hold time)\n", buttonMaps[i].label);
      }
      
      if (buttonMaps[i].action == 4) {
        // Keyboard repeat - send continuously
        uint8_t keys[6] = {(uint8_t)(buttonMaps[i].keyCode & 0xFF), 0, 0, 0, 0, 0};
        usbSendKeyboardReport(buttonMaps[i].keyModifier, keys);
      } else if (buttonMaps[i].action == 5) {
        // Media repeat - send continuously
        usbSendConsumer(buttonMaps[i].mediaCode);
      } else if ((buttonMaps[i].action == 1 || buttonMaps[i].action == 2) && buttonMaps[i].passToUsb) {
        // GPIO/LED with USB forwarding - send immediately without hold time wait
        uint16_t consumerCode = buttonMaps[i].payload[0] | (buttonMaps[i].payload[1] << 8);
        usbSendConsumerMomentary(consumerCode);
      }
    }
  }
  
  // Check TTL timers for auto-revert
  for (int i = 0; i < MAX_BUTTON_MAPS; i++) {
    if (ttlTimers[i].active && buttonMaps[i].active) {
      unsigned long elapsed = millis() - ttlTimers[i].startTime;
      if (elapsed >= buttonMaps[i].ttlMs) {
        ttlTimers[i].active = false;
        
        if (buttonMaps[i].action == 1) {
          // GPIO: Revert based on gpioTtlBehavior setting
          if (buttonMaps[i].gpioTtlBehavior == 1) {
            // Float mode - set pin as input with no pull resistors
            pinMode(buttonMaps[i].gpioPin, INPUT);
            Serial.printf("GPIO%d TTL expired -> FLOAT (INPUT)\n", buttonMaps[i].gpioPin);
          } else {
            // Switch to opposite state (default behavior)
            if (buttonMaps[i].gpioMode == 0) {
              // Toggle mode - turn OFF (LOW) after TTL expires
              gpioStates[i] = false;
              digitalWrite(buttonMaps[i].gpioPin, LOW);
              Serial.printf("GPIO%d TTL expired -> LOW\n", buttonMaps[i].gpioPin);
            } else if (buttonMaps[i].gpioMode == 1) {
              // Pull HIGH mode - revert to LOW
              digitalWrite(buttonMaps[i].gpioPin, LOW);
              gpioStates[i] = false;
              Serial.printf("GPIO%d TTL expired -> LOW\n", buttonMaps[i].gpioPin);
            } else if (buttonMaps[i].gpioMode == 2) {
              // Pull LOW mode - revert to HIGH
              digitalWrite(buttonMaps[i].gpioPin, HIGH);
              gpioStates[i] = true;
              Serial.printf("GPIO%d TTL expired -> HIGH\n", buttonMaps[i].gpioPin);
            }
          }
        } else if (buttonMaps[i].action == 2) {
          // LED: Turn off
          pixel.setPixelColor(0, pixel.Color(0, 0, 0));
          pixel.show();
          Serial.println("LED TTL expired -> OFF");
        }
      }
    }
  }
  
  // Auto-reconnect to bonded remote if disconnected (like Windows does)
  // ONLY runs if device has been successfully paired/bonded at least once
  if (!isConnected && !isConnecting && !connectRunning && !scanRunning) {
    // Check if we have a saved device AND confirmed bonds in NVS
    int numBonds = NimBLEDevice::getNumBonds();
    if (savedAddr.length() > 0 && numBonds > 0) {
      unsigned long now = millis();
      
      // Use faster scan interval (2 seconds) for first 5 attempts after boot
      unsigned long scanInterval = AUTO_SCAN_INTERVAL;
      if (firstBootAfterPairing && fastScanAttempts < MAX_FAST_SCANS) {
        scanInterval = 2000; // Fast scan for first 5 attempts
      }
      
      // Scan periodically for the bonded remote
      if (now - lastAutoScanTime > scanInterval) {
        lastAutoScanTime = now;
        
        if (firstBootAfterPairing && fastScanAttempts < MAX_FAST_SCANS) {
          fastScanAttempts++;
          Serial.printf("Fast auto-scan #%d/%d for bonded remote: %s\n", fastScanAttempts, MAX_FAST_SCANS, savedAddr.c_str());
        } else {
          Serial.printf("Auto-scanning for bonded remote: %s\n", savedAddr.c_str());
          firstBootAfterPairing = false; // Disable fast scan after max attempts
        }
        
        // Quick 3-second scan
        NimBLEScan* pAutoScan = NimBLEDevice::getScan();
        if (pAutoScan) {
          pAutoScan->setActiveScan(true);
          pAutoScan->setInterval(100);
          pAutoScan->setWindow(99);
          
          NimBLEScanResults results = pAutoScan->start(3, false);
          int foundCount = results.getCount();
          Serial.printf("Auto-scan found %d devices\n", foundCount);
          
          // Check if our saved device is advertising
          for (int i = 0; i < foundCount; i++) {
            NimBLEAdvertisedDevice device = results.getDevice(i);
            String addr = device.getAddress().toString().c_str();
            String name = device.getName().c_str();
            
            Serial.printf("  Device %d: %s (%s)\n", i+1, addr.c_str(), name.c_str());
            
            if (addr.equalsIgnoreCase(savedAddr)) {
              Serial.println(">>> Found bonded remote advertising - connecting immediately!");
              pAutoScan->stop();
              pAutoScan->clearResults();
              
              // Connect immediately - don't wait for next loop iteration
              startConnectTask(savedAddr, savedName);
              return;
            }
          }
          
          Serial.println("Bonded remote not found in this scan");
          pAutoScan->clearResults();
        } else {
          Serial.println("ERROR: Could not get scan object");
        }
      }
    }
  }
}
