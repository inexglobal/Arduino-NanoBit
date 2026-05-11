#include <Nanobit.h>
void setup() {
  int x = 108;
  oled.text(2, 20, "Value = %d",x);
  oled.show();
}
void loop() {}
