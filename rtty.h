#include "Arduino.h"

class RTTYTransmitter {
public:
  void begin(bool keepTXOn);
  
  bool push(char c);

  void tick();

private:
  bool keepTXOn;
};


