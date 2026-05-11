#include <Nanobit.h>
void setup() {}
void loop() {
  oled.text(0, 0, "Hello");
  oled.show();
  delay(3000);
  oled.clear();
  oled.show();
  delay(3000);
}