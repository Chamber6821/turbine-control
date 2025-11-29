#include <LiquidCrystal_I2C.h> // LCD library for I2C
#include <limits.h>

class Lcd {
public:
  virtual ~Lcd() = default; // Cleanup
  virtual LiquidCrystal_I2C &lcd() const = 0;
};

// Wraps LiquidCrystal_I2C instance
class LcdWrap : public Lcd { 
  LiquidCrystal_I2C &_lcd; // Reference to LCD
public:
  LcdWrap(LiquidCrystal_I2C &lcd)
      : _lcd(lcd) {} // Constructor
  LiquidCrystal_I2C &lcd() const override {
    return _lcd; // Override
  }
};

class LcdSlice : public Lcd { // Represents LCD section
  Lcd &_lcd; // Reference to Lcd
  int x, y; // Cursor position
public:
  LcdSlice(Lcd &lcd, int x, int y)
      : _lcd(lcd), x(x), y(y) {} // Init
  LiquidCrystal_I2C &lcd() const override {
    auto &lcd = _lcd.lcd(); // Get LCD reference
    lcd.setCursor(x, y); // Set cursor
    return lcd; // Return reference
  }
};

class LcdWithPrefix : public Lcd { // LCD with output prefix
  Lcd &_lcd; // Reference to Lcd
  const char *prefix; // Prefix string
public:
  LcdWithPrefix(Lcd &lcd, const char *prefix)
      : _lcd(lcd), prefix(prefix) {} // Constructor
  LiquidCrystal_I2C &lcd() const override {
    auto &lcd = _lcd.lcd(); // Get LCD reference
    lcd.print(prefix); // Print prefix
    return lcd; // Return reference
  }
};

class SpiDevice { // Abstract class for SPI devices
public:
  virtual ~SpiDevice() = default; // Cleanup
  virtual void select() const = 0; // Pure virtual select
  virtual uint8_t read() const = 0; // Pure virtual read
  virtual void write(uint8_t data) const = 0; // Pure write
  virtual void unselect() const = 0; // Pure unselect
};

class RootDevice : public SpiDevice { // Root SPI device
  int clockPin; // Clock pin
  int dataPin; // Data pin
  int selectorPin; // Chip select pin

public:
  RootDevice(int clockPin, int dataPin, int selectorPin)
      : clockPin(clockPin), dataPin(dataPin),
        selectorPin(selectorPin) {
    pinMode(clockPin, OUTPUT); // Set clock as output
    digitalWrite(clockPin, LOW); // Init clock low
    pinMode(selectorPin, OUTPUT); // Set select as output
    digitalWrite(selectorPin, HIGH); // Deselect device
  }
  void select() const override {
    digitalWrite(selectorPin, LOW); // Select device
  }

  uint8_t read() const override { // Read from SPI
    pinMode(dataPin, INPUT); // Set data as input
    return shiftIn(dataPin, clockPin, MSBFIRST); 
  }

  void write(uint8_t data) const override { // Write to SPI
    pinMode(dataPin, OUTPUT); // Set data as output
    shiftOut(dataPin, clockPin, MSBFIRST, data);
  }

  void unselect() const override {
    digitalWrite(selectorPin, HIGH); // Deselect device
  }
};

// SPI device from hub
class SpiDeviceFromHub : public SpiDevice {
  int port; // Hub port number
  SpiDevice &hubDevice; // Reference to hub

public:
  SpiDeviceFromHub(int port, SpiDevice &hubDevice)
      : port(port), hubDevice(hubDevice) {} // Init
  void select() const override { // Select device
    hubDevice.select(); // Select hub
    hubDevice.write(~(1 << port)); // Select device
    hubDevice.unselect(); // Unselect hub
  }

  uint8_t read() const override {
    return hubDevice.read();
  }

  void write(uint8_t data) const override {
    hubDevice.write(data);
  }

  void unselect() const override { // Unselect device
    hubDevice.select(); // Select hub
    hubDevice.write(0xFF); // Deselect all
    hubDevice.unselect(); // Unselect hub
  }
};

class Led { // Abstract LED control class
public:
  virtual ~Led() = default; // Cleanup
  virtual void on() const = 0; // Turn on LED
  virtual void off() const = 0; // Turn off LED
};

class PanelLed : public Led { // LED on a panel
  SpiDevice &device; // SPI device reference
  mutable uint8_t &state; // Mutable LED state
  uint8_t index; // LED index

public:
  PanelLed(SpiDevice &device, uint8_t &state, uint8_t index)
      : device(device), state(state), index(index) {}

  void on() const override { // Turn LED on
    state |= (1 << index); // Set bit
    device.select(); // Select device
    device.write(state); // Update state
    device.unselect(); // Unselect device
    Serial.print(state, BIN);
    Serial.print(" - on ");
    Serial.print(index);
    Serial.print("\n");
  }

  void off() const override { // Turn LED off
    state &= ~(1 << index); // Clear bit
    device.select(); // Select device
    device.write(state); // Update state
    device.unselect(); // Unselect device
    Serial.print(state, BIN);
    Serial.print(" - off ");
    Serial.print(index);
    Serial.print("\n");
  }
};

class LedPanel { // Panel of LEDs
  SpiDevice &device; // SPI device reference
  mutable uint8_t state = 0; // State of LEDs

public:
  LedPanel(SpiDevice &device) : device(device) {} // Init

  PanelLed led(int index) const { // Get LED by index
    return { device, state, index }; // Return PanelLed
  }
};

class Device { // Abstract device class
public:
  virtual ~Device() = default; // Cleanup
  virtual int update() const = 0; // Update device
};

class IndicatorLed { // Indicator LED
  Led &led; // LED reference
  int _min, _max; // Range

public:
  IndicatorLed(Led &led, int min, int max)
      : led(led), _min(min), _max(max) {} // Init

  void update(int value) const { // Update LED state
    Serial.print(_min);
    Serial.print(" <= ");
    Serial.print(value);
    Serial.print(" <= ");
    Serial.print(_max);
    Serial.print("\n");
    if (_min <= value && value <= _max) 
      led.on(); // Turn on LED
    else 
      led.off(); // Turn off LED
  }
};

// Thermocouple device
class Termocouple : public Device { 
  SpiDevice &termocouple; // SPI device reference
  Lcd &lcd; // LCD reference
  IndicatorLed &red; // Red indicator
  IndicatorLed &green; // Green indicator

public:
  Termocouple(SpiDevice &termocouple, Lcd &lcd,
    IndicatorLed &red, IndicatorLed &green)
      : termocouple(termocouple), lcd(lcd),
        red(red), green(green) {} // Init

  int update() const override { // Update method
    termocouple.select(); // Select device
    auto tempHigh = termocouple.read(); // Read high byte
    auto tempLow = termocouple.read(); // Read low byte
    termocouple.unselect(); // Unselect device
    // Calculate temperature
    auto temp = (tempHigh << 3) | (tempLow >> 5);
    auto lcdPlace = lcd.lcd(); // Get LCD reference
    lcdPlace.print(temp); // Print temperature
    lcdPlace.print("C"); // Print degree
    red.update(temp); // Update red indicator
    green.update(temp); // Update green indicator
    return temp; // Return temperature
  }
};

class Tachometer : public Device { // Tachometer device
  int pin; // Pin for input
  Lcd &lcd; // LCD reference
  IndicatorLed &red; // Red indicator
  IndicatorLed &green; // Green indicator
  int rpm = -1; // RPM value

public:
  Tachometer(int pin, Lcd &lcd, IndicatorLed &red,
    IndicatorLed &green)
      : pin(pin), lcd(lcd), red(red), green(green) { // Init
    pinMode(pin, INPUT); // Set pin as output
  }

  int update() const override { // Update method
    auto lcdPlace = lcd.lcd(); // Get LCD reference
    if (rpm < 0) {
      lcdPlace.print("Error!");
    } else {
      lcdPlace.print(rpm); // Print RPM
      lcdPlace.print("RPM"); // Print degree
    }
    red.update(rpm); // Update red indicator
    green.update(rpm); // Update green indicator
    return rpm; // Return RPM
  }

  // Measure RPM
  unsigned long measure(unsigned long timeout = 100) { 
    auto start = millis(); // Record start time
    auto startState = digitalRead(pin); // Initial state
    // Wait for change
    while (digitalRead(pin) == startState) 
      if (millis() - start >= timeout) { // Check timeout
        rpm = -1; // Set RPM to -1
        return millis() - start; // Return elapsed time
      } 
    auto change1 = millis(); // First change time
    // Wait for change
    while (digitalRead(pin) != startState) 
      if (millis() - start >= timeout) { // Check timeout
        rpm = -1; // Set RPM to -1
        return millis() - start; // Return elapsed time
      } 
    // Wait for next change
    while (digitalRead(pin) == startState) 
      if (millis() - start >= timeout) { // Check timeout
        rpm = -1; // Set RPM to -1
        return millis() - start; // Return elapsed time
      } 
    auto change2 = millis(); // Second change time
    auto period = change2 - change1; // Calculate period
    rpm = 1000 / period * 60; // Calculate RPM
    return millis() - start; // Return elapsed time
  }

  unsigned long superMeasure(unsigned long timeout = 100) {
    long ms[5] = {0};
    unsigned long del = 0;
    for (auto &el : ms) {
      Serial.print("MEASURE!\n");
      del += measure(timeout);
      el = rpm;
      Serial.print("RPM: ");
      Serial.print(el);
      Serial.print("\n");
    }
    long acc = 0;
    int vals = 0;
    Serial.print("SUM!\n");
    for (auto el : ms) {
      if (el >= 0) {
        acc += el;
        vals++;
      }
    }
    Serial.print("CALC!\n");
    if (vals == 0) rpm = -1;
    else rpm = acc / vals;
    Serial.print("RETURN!\n");
    return del;
  }
};

int main() { // Main function
  init(); // Initialize system

  Serial.begin(9600);

  LiquidCrystal_I2C lcd(0x27, 16, 2); // Create LCD instance
  lcd.init(); // Init the LCD
  lcd.backlight(); // Turn on backlight

  auto lcdWrap = LcdWrap{lcd}; // Wrap LCD instance

  // Create LCD slices for display areas
  auto mdLcdSlice = LcdSlice{lcdWrap, 0, 0}; // Slc for mode
  auto tfLcdSlice = LcdSlice{lcdWrap, 8, 0}; // Slc for TF
  auto twLcdSlice = LcdSlice{lcdWrap, 0, 1}; // Slc for TW
  auto tsLcdSlice = LcdSlice{lcdWrap, 8, 1}; // Slc for TS
  auto lsLcdSlice = LcdSlice{lcdWrap, 0, 1}; // Slc for TOS

  // Create LCDs with prefixes for labeling
  auto mdLcd = LcdWithPrefix{mdLcdSlice, "M:"};  // MD disp
  auto tfLcd = LcdWithPrefix{tfLcdSlice, "TF:"}; // TF disp
  auto twLcd = LcdWithPrefix{twLcdSlice, "TW:"}; // TW disp
  auto tsLcd = LcdWithPrefix{tsLcdSlice, "TS:"}; // TS disp
  auto lsLcd = LcdWithPrefix{lsLcdSlice, "TOS:"};// TOS disp

  auto root = RootDevice{12, 13, 11}; // Create root device
  // Create SPI devices from hub
  auto dev0 = SpiDeviceFromHub{0, root};
  auto dev1 = SpiDeviceFromHub{1, root};
  auto dev2 = SpiDeviceFromHub{2, root};
  auto dev3 = SpiDeviceFromHub{3, root};

  auto leds = LedPanel{dev3}; // Create LED panel

  // Initialize thermocouple indicators
  auto tfRed = leds.led(7); // Red LED for TF
  auto tfGreen = leds.led(0); // Green LED for TF
  auto tfRedIndicator = IndicatorLed{tfRed, 0, 200};
  auto tfGreenIndicator = IndicatorLed{tfGreen, 201, 700};

  // Create thermocouple for fire
  auto termocoupleFire = Termocouple{
    dev0, tfLcd, tfRedIndicator, tfGreenIndicator
  };

  // Initialize water thermocouple indicators
  auto twRed = leds.led(6); // Red LED for TW
  auto twGreen = leds.led(1); // Green LED for TW
  auto twRedIndicator = IndicatorLed{twRed, 111, 200};
  auto twGreenIndicator = IndicatorLed{twGreen, 0, 110};

  // Create thermocouple for water
  auto termocoupleWater = Termocouple{
    dev1, twLcd, twRedIndicator, twGreenIndicator
  };

  // Initialize steam thermocouple indicators
  auto tsRed = leds.led(5); // Red LED for TS
  auto tsGreen = leds.led(2); // Green LED for TS
  auto tsRedIndicator = IndicatorLed{tsRed, 0, 110};
  auto tsGreenIndicator = IndicatorLed{tsGreen, 111, 300};

  // Create thermocouple for steam
  auto termocoupleSteam = Termocouple{
    dev2, tsLcd, tsRedIndicator, tsGreenIndicator
  };

  // Initialize tachometer indicators
  auto lsRed = leds.led(4); // Red LED for tachometer
  auto lsGreen = leds.led(3); // Green LED for tachometer
  auto lsRedIndicator = IndicatorLed{lsRed, INT_MIN, 1000};
  auto lsGreenIndicator = IndicatorLed{lsGreen, 1001, INT_MAX};

  // Create tachometer
  auto tachometer = Tachometer{
    A2, lsLcd, lsRedIndicator, lsGreenIndicator
  };


  for (int i = 0; i < 8; i++) {
    leds.led(i).on();
    delay(100);
  }
  for (int i = 0; i < 8; i++) {
    leds.led(i).off();
    delay(100);
  }

  while (true) { // Infinite loop for updates
    delay(2000); // Wait for 2 seconds
    lcd.clear(); // Clear the LCD
    mdLcd.lcd().print("PAGE1"); // Display page indicator
    termocoupleFire.update(); // Update fire thermocouple
    termocoupleWater.update(); // Update water thermocouple
    termocoupleSteam.update(); // Update steam thermocouple

    // Wait delay between pages
    long measureDelay = tachometer.superMeasure(500);
    Serial.print("Delay ");
    Serial.print(max(1, 2000 - measureDelay));
    Serial.print("ms\n");
    delay(max(1, 2000 - measureDelay)); 
    lcd.clear(); // Clear the LCD
    mdLcd.lcd().print("PAGE2"); // Display page indicator
    Serial.print("Start calc\n");
    tachometer.update(); // Update tachometer display
    Serial.print("End calc\n");
  }
}
