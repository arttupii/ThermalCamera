#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

//low/max range of the sensor
int minTemp = 22;
int maxTemp = 34;

//camColors from https://github.com/adafruit/Adafruit_AMG88xx/blob/master/examples/thermal_cam_interpolate/thermal_cam_interpolate.ino
const uint16_t camColors[] = {0x480F,
                              0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010,
                              0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810,
                              0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811, 0x0811, 0x0011, 0x0011,
                              0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2,
                              0x00B2, 0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192,
                              0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273,
                              0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373,
                              0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
                              0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574,
                              0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571, 0x0591, 0x0591,
                              0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD,
                              0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9,
                              0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7, 0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5,
                              0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621,
                              0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640,
                              0x1E40, 0x1E40, 0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60,
                              0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
                              0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0,
                              0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0,
                              0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0,
                              0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0,
                              0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480,
                              0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40,
                              0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200,
                              0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0,
                              0xF080, 0xF060, 0xF040, 0xF020, 0xF800,
                             };

Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

#define cs   10
#define rst  9
#define dc   8


// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define WHITE           0xFFFF

Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, rst);

void setCursorAndClean(int line, int row = 0, int16_t color = WHITE) {
  display.setCursor(65 + row, line * 10);
  display.setTextColor(color);
  display.fillRect(65, line * 10, display.width() - 65, 10, BLACK);
}

class Key {
  public:
    Key(int key) {
      this->key = key;
      last_state = false;
    }
    ~Key() {}
    bool isPressed(long t = 0) {
      bool c = !digitalRead(key);
      bool ret = false;
      if (c == true && last_state == false) {
        if (t > 0) {
          tick = millis();
        } else {
          ret = true;
        }
      }
      if (t > 0 && (millis() - tick) > t && c) {
        ret = true;
        tick = millis();
      }
      if (t > 0 && c == false) {
        tick = millis();
      }
      last_state = c;
      return ret;
    }
  private:
    int key;
    bool last_state;
    long tick;
};

#define KEY_MENU 2
#define KEY_RIGTH 4
#define KEY_LEFT 3

void setup(void) {
  Serial.begin(9600);
  display.begin();

  if (!amg.begin()) {
    Serial.println("Could not find a valid AMG88xx sensor");
    while (1);
  }
  display.fillScreen(BLACK);

  pinMode(5, OUTPUT);
  pinMode(KEY_LEFT, INPUT);
  pinMode(KEY_RIGTH, INPUT);
  pinMode(KEY_MENU, INPUT);
  digitalWrite(KEY_LEFT , HIGH);
  digitalWrite(KEY_RIGTH, HIGH);
  digitalWrite(KEY_MENU, HIGH);
  digitalWrite(5, LOW);
}

inline uint16_t getColorIndex(int x, int y) {
  if (x < 0 || x >= 8) {
    x = 7;
  }
  if (y < 0 || y >= 8) {
    y = 7;
  }
  uint16_t colorIndex = map(pixels[x + y * 8], minTemp, maxTemp, 0, 255);
  if (colorIndex > 255) {
    colorIndex = 255;
  }

  return colorIndex;
}

uint16_t color8x8 [8 * 8];

bool interpolationEnabled = true;

void loop() {
  amg.readPixels(pixels);
  int16_t x = 0;
  int16_t y = 0;
  for (int16_t i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {


    if (interpolationEnabled) {
      //Interpolation START... 8x8 pixels
      //   c0------------------c1
      //    |                  |
      //    |                  |
      //    |                  |
      //    |                  |
      //    |                  |
      //    |                  |
      //   c2------------------c3
      x = i % 8;
      y = i / 8;

#if 0
      int16_t c0 = getColorIndex(x, y);
      int16_t c1 = getColorIndex(x + 1, y);
      int16_t c2 = getColorIndex(x, y + 1);
      int16_t c3 = getColorIndex(x + 1, y + 1);



      for (y = 0; y < 8; y++) {
        uint16_t y_0 = ((c2 - c0) / 7) * (y) +  c0;
        uint16_t y_1 = ((c3 - c1) / 7) * (y) +  c1;
        for (x = 0; x < 8; x++) {
          uint16_t f = ((y_0 / (7)) * (7 - x) + (y_1 / (7)) * (x));
          color8x8[x + y * 8] = camColors[f];
        }
      }
      //Interpolation END
    } else {
      uint16_t colorIndex = map(pixels[i], minTemp, maxTemp, 0, 255);
      if (colorIndex > 255) {
        colorIndex = 255;
      }
      for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
          color8x8[x + y * 8] = camColors[colorIndex];
        }
      }
    }
#else
      uint8_t c0 = getColorIndex(x, y);
      uint8_t c1 = getColorIndex(x + 1, y);
      uint8_t c2 = getColorIndex(x, y + 1);
      uint8_t c3 = getColorIndex(x + 1, y + 1);

      for (y = 0; y < 8; y++) {
        uint8_t c00 = c0 - (c0 - c2) / 8 * (y);
        uint8_t c11 = c1 - (c1 - c3) / 8 * (y);

        for (x = 0; x < 8; x++) {
          color8x8[x + y * 8] = camColors[c00 - (c00 - c11) / 8 * (x)];
        }
      }
      //Interpolation END
    } else {
      uint16_t colorIndex = map(pixels[i], minTemp, maxTemp, 0, 255);
      if (colorIndex > 255) {
        colorIndex = 255;
      }
      for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++) {
          color8x8[x + y * 8] = camColors[colorIndex];
        }
      }
    }
#endif

    int y0 = (i / 8) * 8;
    int x0 = (i % 8) * 8;

    //Write 8x8 box to screen
    display.startWrite();
    display.setAddrWindow(x0, y0, 8, 8);
    display.writePixels(color8x8, sizeof(color8x8) / sizeof(uint16_t));
    display.endWrite();
  }


  static Key menu_key(KEY_MENU);
  static Key rigth_key(KEY_RIGTH);
  static Key left_key(KEY_LEFT);

  static bool inMenu = false;
  if (inMenu == true && menu_key.isPressed(2000)) {
    inMenu = false;
  } else if (inMenu == false && menu_key.isPressed(2000)) {
    menu(true);
    inMenu = true;
  } else if (inMenu) {
    menu(false);
  } else {
    updateInfoText();
    if (rigth_key.isPressed()) {
      interpolationEnabled = !interpolationEnabled;
    }
    if (left_key.isPressed()) {
      autoCalib();
    }
  }
}

void autoCalib() {
  float min = 255;
  float max = -255;
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (pixels[i] > max) {
      max = pixels[i];
    }
    if (pixels[i] < min) {
      min = pixels[i];
    }
  }
  minTemp = min;
  maxTemp = max;
  if (maxTemp - minTemp < 10) {
    int fix = (maxTemp - minTemp) / 2;
    if (fix < 5) fix = 5;

    minTemp = minTemp - fix;
    maxTemp = maxTemp + fix;
  }
}

bool menu(bool initialize) {
  static int index = 0;
  bool updateValues = initialize;
  do {
    if (initialize || updateValues) {
      if (initialize) {
        display.fillRect(65, 0, display.width() - 65, display.height(), BLACK);

        setCursorAndClean(2);
        display.print("MinT:");

        setCursorAndClean(4);
        display.print("MaxT:");
      }

      index == 2 ? setCursorAndClean(3, 5, RED) : setCursorAndClean(3, 5);
      display.print(minTemp);

      index == 3 ? setCursorAndClean(5, 5, RED) : setCursorAndClean(5, 5);
      display.print(maxTemp);

      index == 0 ? setCursorAndClean(0, 0, RED) : setCursorAndClean(0, 0);
      display.print("Reset");

      index == 1 ? setCursorAndClean(1, 0, RED) : setCursorAndClean(1, 0);
      display.print("Auto");

      initialize = false;
    }

    static Key menu_key(KEY_MENU);
    static Key menu_rigth(KEY_RIGTH);
    static Key menu_left(KEY_LEFT);

    updateValues = true;
    if (menu_key.isPressed()) {
      index++;
      if (index > 3) {
        index = 0;
      }
    } else if (menu_rigth.isPressed()) {
      if (index == 1) { //Autocalibration
        autoCalib();
      }
      if (index == 0) { //Set dafaults
        minTemp = 22;
        maxTemp = 34;
      }
      if (index == 2) {
        if (maxTemp > minTemp + 5) {
          minTemp++;
        }
      }
      if (index == 3) {
        maxTemp++;
      }
    } else if (menu_left.isPressed()) {
      if (index == 2) {
        minTemp--;
      }
      if (index == 3) {
        if (maxTemp > minTemp + 5) {
          maxTemp--;
        }
      }
    } else {
      updateValues = false;
    }
    if (minTemp < 0) {
      minTemp = 0;
    }
    if (maxTemp > 80) {
      maxTemp = 80;
    }
  } while (updateValues);
  return true;
}

void updateInfoText() {
  static long t = millis()+1000;
  if (millis() - t < 1000) {
    return;
  }
  t = millis();

  float max = -255, min = 255, center = 22;
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    if (pixels[i] < min) {
      min = pixels[i];
    }
    if (pixels[i] > max) {
      max = pixels[i];
    }
  }
  setCursorAndClean(0, 0, RED);
  display.print("Max:");
  setCursorAndClean(1, 0, RED);
  display.print(max);
  setCursorAndClean(2, 0, BLUE);
  display.print("Min:");
  setCursorAndClean(3, 0, BLUE);
  display.print(min);
  setCursorAndClean(4);
  display.print("-----");
  setCursorAndClean(5);
  display.print((pixels[28] + pixels[29] + pixels[36] + pixels[37]) / 4);
}


