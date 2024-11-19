#include <LiquidCrystal_I2C.h> // Include the LCD library for I2C

class Lcd {
public:
  virtual ~Lcd() = default; // Virtual destructor for proper cleanup
  virtual LiquidCrystal_I2C &lcd() const = 0; // Pure virtual method
};

class LcdWrap : public Lcd { // Wraps an instance of LiquidCrystal_I2C
  LiquidCrystal_I2C &_lcd; // Reference to the LCD object
public:
  // Constructor initializes
  LcdWrap(LiquidCrystal_I2C &lcd) : _lcd(lcd) {}
  LiquidCrystal_I2C &lcd() const override { return _lcd; } // Override
};

class LcdSlice : public Lcd { // Represents a section of the LCD
  Lcd &_lcd; // Reference to a Lcd object
  int x, y; // Cursor position on the LCD
public:
  LcdSlice(Lcd &lcd, int x, int y) : _lcd(lcd), x(x), y(y) {} // Init
  LiquidCrystal_I2C &lcd() const override { // Override lcd method
    auto &lcd = _lcd.lcd(); // Get LCD reference
    lcd.setCursor(x, y); // Set cursor position
    return lcd; // Return LCD reference
  }
};

class LcdWithPrefix : public Lcd { // LCD with a prefix for output
  Lcd &_lcd; // Reference to a Lcd object
  const char *prefix; // Prefix string
public:
  LcdWithPrefix(Lcd &lcd, const char *prefix)
      : _lcd(lcd), prefix(prefix) {} // Constructor initializes
  LiquidCrystal_I2C &lcd() const override { // Override lcd method
    auto &lcd = _lcd.lcd(); // Get LCD reference
    lcd.print(prefix); // Print the prefix
    return lcd; // Return LCD reference
  }
};

class SpiDevice { // Abstract class for SPI devices
public:
  virtual ~SpiDevice() = default; // Virtual destructor
  virtual void select() const = 0; // Pure virtual select method
  virtual uint8_t read() const = 0; // Pure virtual read method
  virtual void write(uint8_t data) const = 0; // Pure write
  virtual void unselect() const = 0; // Pure unselect method
};

class RootDevice : public SpiDevice { // Represents a root SPI device
  int clockPin; // SPI clock pin
  int dataPin; // SPI data pin
  int selectorPin; // Chip select pin

public:
  RootDevice(int clockPin, int dataPin, int selectorPin)
      : clockPin(clockPin), dataPin(dataPin),
        selectorPin(selectorPin) {
    pinMode(clockPin, OUTPUT); // Set clock pin as output
    digitalWrite(clockPin, LOW); // Initialize clock pin low
    pinMode(selectorPin, OUTPUT); // Set select pin as output
    digitalWrite(selectorPin, HIGH); // Deselect device
  }
  // Select device
  void select() const override { digitalWrite(selectorPin, LOW); } 

  uint8_t read() const override { // Read data from SPI
    pinMode(dataPin, INPUT); // Set data pin as input
    return shiftIn(dataPin, clockPin, MSBFIRST); // Read data
  }

  void write(uint8_t data) const override { // Write data to SPI
    pinMode(dataPin, OUTPUT); // Set data pin as output
    shiftOut(dataPin, clockPin, MSBFIRST, data); // Send data
  }

  // Deselect device
  void unselect() const override { digitalWrite(selectorPin, HIGH); } 
};

class SpiDeviceFromHub : public SpiDevice { // SPI device from a hub
  int port; // Port number on the hub
  SpiDevice &hubDevice; // Reference to the hub device

public:
  SpiDeviceFromHub(int port, SpiDevice &hubDevice)
      : port(port), hubDevice(hubDevice) {} // Init port and hub
  void select() const override { // Override select
    hubDevice.select(); // Select the hub
    hubDevice.write(~(1 << port)); // Select the device on hub
    hubDevice.unselect(); // Unselect hub
  }
  // Read data
  uint8_t read() const override { return hubDevice.read(); } 
  // Write data
  void write(uint8_t data) const override { hubDevice.write(data); } 
  void unselect() const override { // Override unselect
    hubDevice.select(); // Select the hub
    hubDevice.write(0xFF); // Deselect all devices
    hubDevice.unselect(); // Unselect hub
  }
};

class Led { // Abstract class for LED control
public:
  virtual ~Led() = default; // Virtual destructor
  virtual void on() const = 0; // Pure virtual method to turn on LED
  virtual void off() const = 0; // Pure virtual method to turn off LED
};

class PanelLed : public Led { // Represents an LED on a panel
  SpiDevice &device; // Reference to SPI device
  mutable uint8_t &state; // Mutable reference to LED state
  uint8_t index; // Index of the LED in the state

public:
  PanelLed(SpiDevice &device, uint8_t &state, uint8_t index)
      : device(device), state(state), index(index) {} // Init

  void on() const override { // Turn the LED on
    state |= (1 << index); // Set the corresponding bit
    device.select(); // Select the device
    device.write(state); // Update the state on device
    device.unselect(); // Unselect device
  }

  void off() const override { // Turn the LED off
    state &= ~(1 << index); // Clear the corresponding bit
    device.select(); // Select the device
    device.write(state); // Update the state on device
    device.unselect(); // Unselect device
  };
};

class LedPanel { // Represents a panel of LEDs
  SpiDevice &device; // Reference to SPI device
  mutable uint8_t state = 0; // Mutable state of all LEDs

public:
  LedPanel(SpiDevice &device)
      : device(device) {} // Initialize device reference

  PanelLed led(int index) const { // Get specific LED by index
    return { device, state, index }; // Return PanelLed instance
  }
};

class Device { // Abstract base class for devices
public:
  virtual ~Device() = default; // Virtual destructor
  virtual int update() const = 0; // Pure virtual update method
};

class IndicatorLed { // Represents an indicator LED
  Led &led; // Reference to an LED
  int _min, _max; // Range for the indicator

public:
  IndicatorLed(Led &led, int min, int max)
      : led(led), _min(min), _max(max) {} // Init

  void update(int value) const { // Update LED state based on value
    if (_min <= value && value <= _max) led.on(); // Turn on LED
    else led.off(); // Turn off LED
  }
};

// Represents a thermocouple device
class Termocouple : public Device { 
  SpiDevice &termocouple; // Reference to SPI device
  Lcd &lcd; // Reference to LCD
  IndicatorLed &red; // Red indicator LED
  IndicatorLed &green; // Green indicator LED

public:
  Termocouple(
    SpiDevice &termocouple,
    Lcd &lcd,
    IndicatorLed &red,
    IndicatorLed &green
  )
      : termocouple(termocouple), lcd(lcd),
        red(red), green(green) {} // Init

  int update() const override { // Override update method
    termocouple.select(); // Select the termocouple device
    auto tempHigh = termocouple.read(); // Read high byte
    auto tempLow = termocouple.read(); // Read low byte
    termocouple.unselect(); // Unselect device
    auto temp = (tempHigh << 3) | (tempLow >> 5); // Calculate temp
    auto lcdPlace = lcd.lcd(); // Get LCD reference
    lcdPlace.print(temp); // Print temperature
    lcdPlace.print("C"); // Print degree symbol
    red.update(temp); // Update red indicator
    green.update(temp); // Update green indicator
    return temp; // Return temperature
  }
};

class Tachometer : public Device { // Represents a tachometer device
  int pin; // Pin for tachometer input
  Lcd &lcd; // Reference to LCD
  IndicatorLed &red; // Red indicator LED
  IndicatorLed &green; // Green indicator LED
  int rpm = -1; // RPM value initialized to -1

public:
  Tachometer(
    int pin,
    Lcd &lcd,
    IndicatorLed &red,
    IndicatorLed &green
  )
      : pin(pin), lcd(lcd),
        red(red), green(green) { // Init values
    pinMode(pin, OUTPUT); // Set pin as output
  }

  int update() const override { // Override update method
    auto lcdPlace = lcd.lcd(); // Get LCD reference
    lcdPlace.print(rpm); // Print RPM value
    lcdPlace.print("C"); // Print degree symbol
    red.update(rpm); // Update red indicator
    green.update(rpm); // Update green indicator
    return rpm; // Return RPM value
  }

  unsigned long measure(unsigned long timeout = 100) { // Measure RPM
    auto start = millis(); // Record start time
    auto startState = digitalRead(pin); // Read initial state
    while (digitalRead(pin) == startState) // Wait for change
      if (millis() - start >= timeout) { // Check timeout
        rpm = -1; // Set RPM to -1 if timeout
        return millis() - start; // Return elapsed time
      } 
    auto change1 = millis(); // Record first change time
    while (digitalRead(pin) != startState) // Wait for state change
      if (millis() - start >= timeout) { // Check timeout
        rpm = -1; // Set RPM to -1 if timeout
        return millis() - start; // Return elapsed time
      } 
    while (digitalRead(pin) == startState) // Wait for next change
      if (millis() - start >= timeout) { // Check timeout
        rpm = -1; // Set RPM to -1 if timeout
        return millis() - start; // Return elapsed time
      } 
    auto change2 = millis(); // Record second change time
    auto period = change2 - change1; // Calculate period
    rpm = 60 * 1000 / period; // Calculate RPM from period
    return millis() - start; // Return elapsed time
  }
};

int main() { // Main function
  init(); // Initialize the system

  LiquidCrystal_I2C lcd(0x27, 16, 2); // Create LCD instance
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the backlight

  auto lcdWrap = LcdWrap{lcd}; // Wrap the LCD instance

  // Create LCD slices for different display areas
  auto mdLcdSlice = LcdSlice{lcdWrap, 0, 0}; // Slice for main display
  auto tfLcdSlice = LcdSlice{lcdWrap, 8, 0}; // Slice for TF display
  auto twLcdSlice = LcdSlice{lcdWrap, 0, 1}; // Slice for TW display
  auto tsLcdSlice = LcdSlice{lcdWrap, 8, 1}; // Slice for TS display
  auto lsLcdSlice = LcdSlice{lcdWrap, 0, 1}; // Slice for TOS display

  // Create LCDs with prefixes for labeling
  auto mdLcd = LcdWithPrefix{mdLcdSlice, "M:"}; // Main display
  auto tfLcd = LcdWithPrefix{tfLcdSlice, "TF:"}; // TF display
  auto twLcd = LcdWithPrefix{twLcdSlice, "TW:"}; // TW display
  auto tsLcd = LcdWithPrefix{tsLcdSlice, "TS:"}; // TS display
  auto lsLcd = LcdWithPrefix{lsLcdSlice, "TOS:"}; // TOS display

  auto root = RootDevice{9, 10, 11}; // Create root device
  // Create SPI devices from hub
  auto dev0 = SpiDeviceFromHub{0, root};
  auto dev1 = SpiDeviceFromHub{1, root};
  auto dev2 = SpiDeviceFromHub{2, root};
  auto dev3 = SpiDeviceFromHub{3, root};

  auto leds = LedPanel{dev3}; // Create LED panel

  // Initialize thermocouple indicators
  auto tfRed = leds.led(0); // Red LED for TF
  auto tfGreen = leds.led(1); // Green LED for TF
  auto tfRedIndicator = IndicatorLed{tfRed, 0, 200}; // Red indicator
  auto tfGreenIndicator = IndicatorLed{tfRed, 200, 700}; // Green

  // Create thermocouple for fire
  auto termocoupleFire = Termocouple{
    dev0, tfLcd, tfRedIndicator, tfGreenIndicator
  };

  // Initialize water thermocouple indicators
  auto twRed = leds.led(2); // Red LED for TW
  auto twGreen = leds.led(3); // Green LED for TW
  auto twRedIndicator = IndicatorLed{twRed, 110, 200}; // Red indicator
  auto twGreenIndicator = IndicatorLed{twRed, 90, 110}; // Green

  // Create thermocouple for water
  auto termocoupleWater = Termocouple{
    dev1, twLcd, twRedIndicator, twGreenIndicator
  };

  // Initialize steam thermocouple indicators
  auto tsRed = leds.led(4); // Red LED for TS
  auto tsGreen = leds.led(5); // Green LED for TS
  auto tsRedIndicator = IndicatorLed{twRed, 0, 110}; // Red indicator
  auto tsGreenIndicator = IndicatorLed{twRed, 110, 300}; // Green

  // Create thermocouple for steam
  auto termocoupleSteam = Termocouple{
    dev2, tsLcd, tsRedIndicator, tsGreenIndicator
  };

  // Initialize tachometer indicators
  auto lsRed = leds.led(6); // Red LED for tachometer
  auto lsGreen = leds.led(7); // Green LED for tachometer
  auto lsRedIndicator = IndicatorLed{twRed, 0, 1000}; // Red indicator
  auto lsGreenIndicator = IndicatorLed{twRed, 1000, 1500}; // Green

  // Create tachometer
  auto tachometer = Tachometer{
    8, lsLcd, lsRedIndicator, lsGreenIndicator
  };

  while (true) { // Infinite loop for updates
    delay(2000); // Wait for 2 seconds
    lcd.clear(); // Clear the LCD
    mdLcd.lcd().print("PAGE1"); // Display page indicator
    termocoupleFire.update(); // Update fire thermocouple
    termocoupleWater.update(); // Update water thermocouple
    termocoupleSteam.update(); // Update steam thermocouple

    delay(2000 - tachometer.measure(2000)); // Wait based on tachometer
    lcd.clear(); // Clear the LCD
    mdLcd.lcd().print("PAGE2"); // Display page indicator
    tachometer.update(); // Update tachometer display
  }
}
