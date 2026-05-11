#include <Nanobit.h>
void setup() {}
void loop() {
  int i;
  for (i = 0; i <= 3; i++) {
    oled.clear();
    oled.mode(i);
    oled.text(0, 0, "Mode = %d", i);
    oled.show();
    delay(3000);
  }
}