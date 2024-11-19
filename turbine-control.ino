#include <LiquidCrystal_I2C.h>

class Lcd {
public:
  virtual ~Lcd() = default;
  virtual LiquidCrystal_I2C &lcd() const = 0;
};

class LcdWrap : public Lcd {
  LiquidCrystal_I2C &_lcd;
public:
  LcdWrap(LiquidCrystal_I2C &lcd) : _lcd(lcd) {}
  LiquidCrystal_I2C &lcd() const override { return _lcd; }
};

class LcdSlice : public Lcd {
  Lcd &_lcd;
  int x, y;
public:
  LcdSlice(Lcd &lcd, int x, int y) : _lcd(lcd), x(x), y(y) {}
  LiquidCrystal_I2C &lcd() const override {
    auto &lcd = _lcd.lcd();
    lcd.setCursor(x, y);
    return lcd;
  }
};

class LcdWithPrefix : public Lcd {
  Lcd &_lcd;
  const char *prefix;
public:
  LcdWithPrefix(Lcd &lcd, const char *prefix)
      : _lcd(lcd), prefix(prefix) {}
  LiquidCrystal_I2C &lcd() const override {
    auto &lcd = _lcd.lcd();
    lcd.print(prefix);
    return lcd;
  }
};

class SpiDevice {
public:
  virtual ~SpiDevice() = default;
  virtual void select() const = 0;
  virtual uint8_t read() const = 0;
  virtual void write(uint8_t data) const = 0;
  virtual void unselect() const = 0;
};

class RootDevice : public SpiDevice {
  int clockPin;
  int dataPin;
  int selectorPin;

public:
  RootDevice(int clockPin, int dataPin, int selectorPin)
      : clockPin(clockPin), dataPin(dataPin), selectorPin(selectorPin) {
    pinMode(clockPin, OUTPUT);
    digitalWrite(clockPin, LOW);
    pinMode(selectorPin, OUTPUT);
    digitalWrite(selectorPin, HIGH);
  }

  void select() const override { digitalWrite(selectorPin, LOW); }

  uint8_t read() const override {
    pinMode(dataPin, INPUT);
    return shiftIn(dataPin, clockPin, MSBFIRST);
  }

  void write(uint8_t data) const override {
    pinMode(dataPin, OUTPUT);
    shiftOut(dataPin, clockPin, MSBFIRST, data);
  }

  void unselect() const override { digitalWrite(selectorPin, HIGH); }
};

class SpiDeviceFromHub : public SpiDevice {
  int port;
  SpiDevice &hubDevice;

public:
  SpiDeviceFromHub(int port, SpiDevice &hubDevice)
      : port(port), hubDevice(hubDevice) {}
  void select() const override {
    hubDevice.select();
    hubDevice.write(~(1 << port));
    hubDevice.unselect();
  }
  uint8_t read() const override { return hubDevice.read(); }
  void write(uint8_t data) const override { hubDevice.write(data); }
  void unselect() const override {
    hubDevice.select();
    hubDevice.write(0xFF);
    hubDevice.unselect();
  }
};

class Led {
public:
  virtual ~Led() = default;
  virtual void on() const = 0;
  virtual void off() const = 0;
};

class PanelLed : public Led {
  SpiDevice &device;
  mutable uint8_t &state;
  uint8_t index;
public:
  PanelLed(SpiDevice &device, uint8_t &state, uint8_t index)
      : device(device), state(state), index(index) {}
  void on() const override {
    state |= (1 << index);
    device.select();
    device.write(state);
    device.unselect();
  }
  void off() const override {
    state &= ~(1 << index);
    device.select();
    device.write(state);
    device.unselect();
  };
};

class LedPanel {
  SpiDevice &device;
  mutable uint8_t state = 0;
public:
  LedPanel(SpiDevice &device)
      : device(device) {}
  PanelLed led(int index) const {
    return { device, state, index };
  }
};

class Device {
public:
  virtual ~Device() = default;
  virtual int update() const = 0;
};

class IndicatorLed {
  Led &led;
  int _min, _max;
public:
  IndicatorLed(Led &led, int min, int max)
      : led(led), _min(min), _max(max) {}
  void update(int value) const {
    if (_min <= value && value <= _max) led.on();
    else led.off();
  }
};

class Termocouple : public Device {
  SpiDevice &termocouple;
  Lcd &lcd;
  IndicatorLed &red;
  IndicatorLed &green;
public:
  Termocouple(
    SpiDevice &termocouple,
    Lcd &lcd,
    IndicatorLed &red,
    IndicatorLed &green
  )
      : termocouple(termocouple), lcd(lcd),
        red(red), green(green) {}
  int update() const override {
    termocouple.select();
    auto tempHigh = termocouple.read();
    auto tempLow = termocouple.read();
    termocouple.unselect();
    auto temp = (tempHigh << 3) | (tempLow >> 5);
    auto lcdPlace = lcd.lcd();
    lcdPlace.print(temp);
    lcdPlace.print("C");
    red.update(temp);
    green.update(temp);
    return temp;
  }
};

class Tachometer : public Device {
  int pin;
  Lcd &lcd;
  IndicatorLed &red;
  IndicatorLed &green;
  int rpm = -1;
public:
  Tachometer(
    int pin,
    Lcd &lcd,
    IndicatorLed &red,
    IndicatorLed &green
  )
      : pin(pin), lcd(lcd),
        red(red), green(green) {
    pinMode(pin, OUTPUT);
  }
  int update() const override {
    auto lcdPlace = lcd.lcd();
    lcdPlace.print(rpm);
    lcdPlace.print("C");
    red.update(rpm);
    green.update(rpm);
    return rpm;
  }
  unsigned long measure(unsigned long timeout = 100) {
    auto start = millis();
    auto startState = digitalRead(pin);
    while (digitalRead(pin) == startState)
      if (millis() - start >= timeout) {
        rpm = -1;
        return millis() - start;
      } 
    auto change1 = millis();
    while (digitalRead(pin) != startState)
      if (millis() - start >= timeout) {
        rpm = -1;
        return millis() - start;
      } 
    while (digitalRead(pin) == startState)
      if (millis() - start >= timeout) {
        rpm = -1;
        return millis() - start;
      } 
    auto change2 = millis();
    auto period = change2 - change1;
    rpm = 60 * 1000 / period;
    return millis() - start;
  }
};

int main() {
  init();

  LiquidCrystal_I2C lcd(0x27, 16, 2);
  lcd.init();
  lcd.backlight();
  auto lcdWrap = LcdWrap{lcd};

  auto mdLcdSlice = LcdSlice{lcdWrap, 0, 0};
  auto tfLcdSlice = LcdSlice{lcdWrap, 8, 0};
  auto twLcdSlice = LcdSlice{lcdWrap, 0, 1};
  auto tsLcdSlice = LcdSlice{lcdWrap, 8, 1};
  auto lsLcdSlice = LcdSlice{lcdWrap, 0, 1};

  auto mdLcd = LcdWithPrefix{mdLcdSlice, "M:"};
  auto tfLcd = LcdWithPrefix{tfLcdSlice, "TF:"};
  auto twLcd = LcdWithPrefix{twLcdSlice, "TW:"};
  auto tsLcd = LcdWithPrefix{tsLcdSlice, "TS:"};
  auto lsLcd = LcdWithPrefix{lsLcdSlice, "TOS:"};

  auto root = RootDevice{9, 10, 11};
  auto dev0 = SpiDeviceFromHub{0, root};
  auto dev1 = SpiDeviceFromHub{1, root};
  auto dev2 = SpiDeviceFromHub{2, root};
  auto dev3 = SpiDeviceFromHub{3, root};

  auto leds = LedPanel{dev3};

  auto tfRed = leds.led(0);
  auto tfGreen = leds.led(1);
  auto tfRedIndicator = IndicatorLed{tfRed, 0, 200};
  auto tfGreenIndicator = IndicatorLed{tfRed, 200, 700};
  auto termocoupleFire = Termocouple{
    dev0, tfLcd, tfRedIndicator, tfGreenIndicator
  };

  auto twRed = leds.led(2);
  auto twGreen = leds.led(3);
  auto twRedIndicator = IndicatorLed{twRed, 110, 200};
  auto twGreenIndicator = IndicatorLed{twRed, 90, 110};
  auto termocoupleWater = Termocouple{
    dev1, twLcd, twRedIndicator, twGreenIndicator
  };

  auto tsRed = leds.led(4);
  auto tsGreen = leds.led(5);
  auto tsRedIndicator = IndicatorLed{twRed, 0, 110};
  auto tsGreenIndicator = IndicatorLed{twRed, 110, 300};
  auto termocoupleSteam = Termocouple{
    dev2, tsLcd, tsRedIndicator, tsGreenIndicator
  };

  auto lsRed = leds.led(6);
  auto lsGreen = leds.led(7);
  auto lsRedIndicator = IndicatorLed{twRed, 0, 1000};
  auto lsGreenIndicator = IndicatorLed{twRed, 1000, 1500};
  auto tachometer = Tachometer{
    8, lsLcd, lsRedIndicator, lsGreenIndicator
  };

  while (true) {
    delay(2000);
    lcd.clear();
    mdLcd.lcd().print("PAGE1");
    termocoupleFire.update();
    termocoupleWater.update();
    termocoupleSteam.update();

    delay(2000 - tachometer.measure(2000));
    lcd.clear();
    mdLcd.lcd().print("PAGE2");
    tachometer.update();
  }
}
