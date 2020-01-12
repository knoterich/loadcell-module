#include <FS.h>
#include <ArduinoJson.h>


#include <jled.h>
#include "HX711.h"
#include <ESP8266WiFi.h>



#define pin_sck 4
#define pin_dt 5

#define pin_blue 13
#define pin_red 14
#define pin_green 12

#define pin_led 15

#define buffer_serial_size 36

HX711 scale;

byte led_ping_pin = pin_blue;
byte led_error = pin_red;
byte led_ack = pin_green;

char server_ip[40] = {};
char server_port[10] = {};
char ssid[32] = {""};
char pass[64] = {""};

float cal_factor = 0;
int dec_places = 2;
int num_samples = 1;
int count_samples = 0;
unsigned int freq_sample = 12500;
int freq_ping = 1000;
int dur_ping = 20;
float measurement = 0;
byte stat_poll = 1;
byte stat_ping = 0;
unsigned int ping_time = 0;
unsigned int ping_time_old = 0;
unsigned int time_zero = 0;

unsigned int cycles = 0;
unsigned int old_cycles = 0;


char buffer_serial[buffer_serial_size];

void setup() {

  pinMode(pin_green, OUTPUT);
  pinMode(pin_red, OUTPUT);
  pinMode(pin_blue, OUTPUT);
  pinMode(pin_led, OUTPUT);
  digitalWrite(pin_green, LOW);
  digitalWrite(pin_red, LOW);
  digitalWrite(pin_blue, LOW);
  digitalWrite(pin_led, LOW);
  Serial.begin(115200);

  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }

  loadConfig();

  scale.begin(pin_dt, pin_sck);

  WiFi.setAutoConnect(false);
  WiFi.mode(WIFI_STA);

}

void loop() {

  while (WiFi.status() != WL_CONNECTED)
  {
    yield();
    Serial.print("Connecting to SSID:");
    Serial.print(ssid);
    Serial.print(" Pass:");
    Serial.print(pass);
    Serial.println(" send ser to interrupt and enter serial config.");

    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED)
    {
      yield();
      Serial.print(".");
      delay(500);
      if (Serial.available())
      {
        WiFi.disconnect();
        break;
      }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("WiFi connected.");
      break;
    }

    if (Serial.available() >= 3)
    {
      Serial.readBytesUntil('\n', buffer_serial, buffer_serial_size);
      if (buffer_serial[0] == 's' && buffer_serial[1] == 'e' && buffer_serial[2] == 'r') //ser starting the TCP/IP and WIFI configuration through Serial
      {
        wifi_param();
      }
    }
  }



  WiFiClient client;
  while (!client.connected())
  {
    yield();
    Serial.print("Connecting to Server:");
    Serial.print(server_ip);
    Serial.print(" Port:");
    Serial.println(server_port);

    while (!client.connected())
    {
      client.connect(server_ip, atoi(server_port));
      Serial.print(".");
      delay(500);
      if (Serial.available())
      {
        break;
      }
    }
    if (client.connected())
    {
      Serial.println("Connection to server established.");
      client.setNoDelay(true);
      break;
    }

    if (Serial.available() >= 3)
    {
      Serial.readBytesUntil('\n', buffer_serial, buffer_serial_size);
      if (buffer_serial[0] == 's' && buffer_serial[1] == 'e' && buffer_serial[2] == 'r') //ser starting the TCP/IP and WIFI configuration through Serial
      {
        wifi_param();
        client.stop();
      }
    }
  }


  while (client.connected())
  {
    yield();
    if (client.available() >= 3)
    {
      char buffer_serial[buffer_serial_size];
      for (int i = 0; i < buffer_serial_size; i++)
      {
        buffer_serial[i] = 0;
      }
      client.readBytesUntil('\n', buffer_serial, buffer_serial_size);
      if (buffer_serial[0] == 't' && buffer_serial[1] == 'a' && buffer_serial[2] == 'r') //taring the scale
      {
        Serial.println(buffer_serial);
        scale.tare();
        client.print("ack#");
      }
      else if (buffer_serial[0] == 'c' && buffer_serial[1] == 'a' && buffer_serial[2] == 'l') //calibrating
      {
        float cal_weight = 0;
        client.print("ack#");
        while (client.available() == 0)
        {
          yield();
        }
        while (client.available() != 0)
        {
          yield();
          client.readBytesUntil('\n', buffer_serial, buffer_serial_size);
          cal_weight = atof(buffer_serial);
        }

        cal_factor = scale.get_units(10);
        cal_factor = cal_factor / cal_weight;
        Serial.print("cal_factor during calibration=");
        Serial.println(cal_factor);
        scale.set_scale(cal_factor);
        //saveConfig();
        client.print("ack#");
      }
      else if (buffer_serial[0] == 'd' && buffer_serial[1] == 'e' && buffer_serial[2] == 'c')
      {
        client.print("ack#");
        while (client.available() == 0)
        {
          yield();
        }
        while (client.available() != 0)
        {
          yield();
          client.readBytesUntil('\n', buffer_serial, buffer_serial_size);
          dec_places = atoi(buffer_serial);
          Serial.println(dec_places);
          //saveConfig();
        }
        client.print("ack#");
      }
      else if (buffer_serial[0] == 's' && buffer_serial[1] == 'e' && buffer_serial[2] == 't')
      {
        scale.set_scale();
        client.print("ack#");
      }
      else if (buffer_serial[0] == 'b' && buffer_serial[1] == 'l' && buffer_serial[2] == 'k')
      {
        digitalWrite(led_ping_pin, HIGH);
        delay(200);
        digitalWrite(led_ping_pin, LOW);
      }
      else if (buffer_serial[0] == 'p' && buffer_serial[1] == 'o' && buffer_serial[2] == 'e')//poe enables polling
      {
        stat_poll = 1;
        client.print("ack#");
      }
      else if (buffer_serial[0] == 'p' && buffer_serial[1] == 'o' && buffer_serial[2] == 'd')//pod disables polling
      {
        stat_poll = 0;
        client.print("ack#");
        Serial.println("pod ack");
      }
      else if (buffer_serial[0] == 'p' && buffer_serial[1] == 'i' && buffer_serial[2] == 'g') //enable blink
      {
        stat_ping = stat_ping ^ 0x01;
        client.print("ack#");
      }
      else if (buffer_serial[0] == 'f' && buffer_serial[1] == 'p' && buffer_serial[2] == 'n')//
      {
        client.print("ack#");
        while (client.available() == 0)
        {
          yield();
        }
        while (client.available() != 0)
        {
          yield();
          client.readBytesUntil('\n', buffer_serial, buffer_serial_size);
          freq_ping = atoi(buffer_serial);
        }
        client.print("ack#");
      }
      else if (buffer_serial[0] == 'd' && buffer_serial[1] == 'p' && buffer_serial[2] == 'n')
      {
        client.print("ack#");
        while (client.available() == 0)
        {
          yield();
        }
        while (client.available() != 0)
        {
          yield();
          client.readBytesUntil('\n', buffer_serial, buffer_serial_size);
          dur_ping = atoi(buffer_serial);
        }
        client.print("ack#");
      }
      else if (buffer_serial[0] == 'f' && buffer_serial[1] == 's' && buffer_serial[2] == 'm')
      {
        Serial.println("ready for setting samplerate");
        Serial.print(buffer_serial);
        Serial.print("//");
        client.print("ack#");
        while (client.available() == 0)
        {
          delay(100);
        }
        client.readBytesUntil('\n', buffer_serial, buffer_serial_size);
        long long_buffer;
        Serial.print(buffer_serial);
        Serial.print("//");
        long_buffer = atol(buffer_serial);
        Serial.println(long_buffer);
        freq_sample = (unsigned int) long_buffer;

        //buffer_serial[0] = '\0';
        //saveConfig();
        client.print("ack#");

      }
      else if (buffer_serial[0] == 'n' && buffer_serial[1] == 's' && buffer_serial[2] == 'm')
      {
        client.print("ack#");
        while (client.available() == 0)
        {
          yield();
        }
        while (client.available() != 0)
        {
          yield();
          client.readBytesUntil('\n', buffer_serial, buffer_serial_size);
          //Serial.print(buffer_serial);
          num_samples = atoi(buffer_serial);
          count_samples = 0;
          //saveConfig();
        }
        client.print("ack#");
      }
      else if (buffer_serial[0] == 'z' && buffer_serial[1] == 'e' && buffer_serial[2] == 'r')
      {
        time_zero = millis();
        old_cycles = micros();
      }
      else if (buffer_serial[0] == 's' && buffer_serial[1] == 'a' && buffer_serial[2] == 'v')
      {

      }
      else
      {
        client.print("inv#");
      }
    }

    if (stat_poll == 1)
    {
      cycles = micros() - old_cycles;
      if (cycles >= freq_sample)
      {
        if (scale.is_ready())
        {
          measurement = measurement + scale.get_units();
          count_samples = count_samples + 1;
          old_cycles = old_cycles + freq_sample;
        }
        else
        {
        }
        if (count_samples == num_samples)
        {
          measurement = measurement / (float) num_samples;
          char outputstring[64];
          outputstring[0] = '\0';
          strcat(outputstring, ":mes:");
          ltoa(millis() - time_zero, &outputstring[strlen(outputstring)], 10);
          strcat(outputstring, ":");
          dtostrf(measurement, 1, dec_places, &outputstring[strlen(outputstring)]);
          strcat(outputstring, ":#");
          client.print(outputstring);
          //Serial.println(outputstring);
          count_samples = 0;
          measurement = 0;
        }

      }
    }

    if (stat_ping == 1)
    {
      if ((millis() - ping_time_old) >= ping_time)
      {
        if (digitalRead(led_ping_pin) == HIGH)
        {
          ping_time = freq_ping - dur_ping;
          ping_time_old = millis();
        }
        else if (digitalRead(led_ping_pin) == LOW)
        {
          ping_time = dur_ping;
          ping_time_old = millis();
          client.print(":blk:");
          client.print(millis() - time_zero);
          client.print(":#");
        }
        digitalWrite(led_ping_pin, !digitalRead(led_ping_pin));

      }
    }

    if (Serial.available() >= 3)
    {
      wifi_param();
    }

    yield();
  }
}

void wifi_param()
{
  char buffer_command[10] = {};
  int buffer_command_size = 10;
  bool setup_loop = 0;
  auto led = JLed(led_ping_pin).Breathe(2000).Forever();
  Serial.print("SSID:");
  Serial.println(ssid);
  Serial.print("Pass:");
  Serial.println(pass);
  Serial.print("Server IP:");
  Serial.println(server_ip);
  Serial.print("Port:");
  Serial.println(server_port);
  while (setup_loop == 0)
  {
    yield();
    Serial.println("Wating for command.");
    while (Serial.available() == 0)
    {
      led.Update();
      yield();
    }
    auto led = JLed(led_ping_pin).Off().Forever().Update();
    Serial.readBytesUntil('\n', buffer_command, buffer_command_size);

    if (buffer_command[0] == 's' && buffer_command[1] == 'i' && buffer_command[2] == 'd') //sid SSID for wifi
    {
      Serial.print("ack, awaiting SSID");
      while (Serial.available() == 0)
      {
        yield();
      }
      int number_of_bytes_received = Serial.readBytesUntil('\n', ssid, 32);
      ssid[number_of_bytes_received] = 0; // add a 0 terminator to the char array
      Serial.print("ack, SSID:");
      Serial.println(ssid);
    }
    else if (buffer_command[0] == 'p' && buffer_command[1] == 'a' && buffer_command[2] == 's') //pas Pass for WIFI
    {
      Serial.print("ack, awaiting Wifi pass.");
      while (Serial.available() == 0)
      {
        yield();
      }
      int number_of_bytes_received = Serial.readBytesUntil('\n', pass, 64);
      pass[number_of_bytes_received] = 0; // add a 0 terminator to the char array
      Serial.print("ack, Pass:");
      Serial.println(pass);
    }
    else if (buffer_command[0] == 'p' && buffer_command[1] == 'o' && buffer_command[2] == 'r') //por port number
    {
      Serial.print("ack, awaiting server port\n");
      while (Serial.available() == 0)
      {
        yield();
      }
      Serial.readBytesUntil('\n', server_port, 10);
      Serial.print("ack, Port number:");
      Serial.print(server_port);
      Serial.print("\n");
    }
    else if (buffer_command[0] == 's' && buffer_command[1] == 'i' && buffer_command[2] == 'p') // sip Server IP
    {
      Serial.print("ack, awaiting server IP\n");
      while (Serial.available() == 0)
      {
        yield();
      }
      Serial.readBytesUntil('\n', server_ip, 40);
      Serial.print("ack, Server IP:");
      Serial.print(server_ip);
      Serial.print("\n");
    }
    else if (buffer_command[0] == 's' && buffer_command[1] == 'a' && buffer_command[2] == 'v') //sav saving settings to non-volatile memory
    {
      Serial.print("ack, saving setup configuration\n");
      saveConfig();
      Serial.print("setup configuration saved");
    }
    else if (buffer_command[0] == 's' && buffer_command[1] == 'r' && buffer_command[2] == 't') //srt starts with the configuration
    {
      setup_loop = 1;

    }
  }
}

bool loadConfig() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, configFile);

  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  strlcpy(ssid,                  // <- destination
          doc["wifi_ssid"],  // <- source
          sizeof(ssid));         // <- destination's capacity
  strlcpy(pass,                  // <- destination
          doc["wifi_pass"],  // <- source
          sizeof(pass));         // <- destination's capacity
  dec_places = doc["dec_places"];
  freq_ping = doc["freq_ping"];
  dur_ping = doc["dur_ping"];
  freq_sample = doc["freq_sample"];
  num_samples = doc["num_samples"];
  cal_factor = doc["cal_factor"];
  strlcpy(server_ip,                  // <- destination
          doc["server"],  // <- source
          sizeof(server_ip));         // <- destination's capacity
  strlcpy(server_port,                  // <- destination
          doc["port"],  // <- source
          sizeof(server_port));         // <- destination's capacity
  scale.set_scale(cal_factor);

  // Close the file (Curiously, File's destructor doesn't close the file)
  configFile.close();
}


bool saveConfig() {
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  StaticJsonDocument<512> doc;

  // Set the values in the document
  doc["wifi_ssid"] = ssid;
  doc["wifi_pass"] = pass;
  doc["server"] = server_ip;
  doc["port"] = server_port;
  doc["dec_places"] = dec_places;
  doc["freq_ping"] = freq_ping;
  doc["dur_ping"] = dur_ping;
  doc["freq_sample"] = freq_sample;
  doc["num_samples"] = num_samples;
  doc["cal_factor"] = cal_factor;

  // Serialize JSON to file
  if (serializeJson(doc, configFile) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  configFile.close();
}