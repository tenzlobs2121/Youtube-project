#include <HardwareSerial.h>

HardwareSerial SerialPort(2); // use UART2

char number[4];
bool isReceiving = false; // Flag to track receiving mode

void setup()
{
  Serial.begin(115200);
  SerialPort.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println();
}

void loop()
{
  if (!isReceiving) {
    SerialPort.print(5678);
    Serial.printf("Sending message from ESP32 to AT32f415 is 5678");
    Serial.printf("\n\r");

    delay(2000); // Delay to allow the transmitted message to be sent

    isReceiving = true; // Enter receiving mode
  }

  if (SerialPort.available() >= 3)
  {
    Serial.print("Received data from AT32F415 is: ");
    SerialPort.readBytes(number, 3);
    number[3] = '\0';
    Serial.println(number);
    Serial.printf("\n\r");

    isReceiving = false; // Exit receiving mode
    delay(3000); // Delay before sending the next message
  }
}
