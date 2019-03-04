#include <SPI.h>

#include <WiFi.h>
#include <WebServer.h>

#include <ESPmDNS.h>

#include <ArduinoJson.h>

#define ADS1299_PIN_RESET 25
#define ADS1299_PIN_DRDY 13

#define ADS1299_PIN_SCK 27
#define ADS1299_PIN_MISO 12
#define ADS1299_PIN_MOSI 32
#define ADS1299_PIN_SS 17

#define OPENBCI_DATA_BUFFER_SIZE 50

#define OPENBCI_NAME "OpenBCI-FFFF"
#define OPENBCI_VERSION "v2.0.5"

#define SOFT_AP_SSID "OpenBCI WiFi AP"
#define SOFT_AP_PASSWORD "abcd1234*"

#define JSON_BUFFER_SIZE 1024

enum ads1299_command : uint8_t
{
    ads1299_command_start = 0x08,
    ads1299_command_stop = 0x0A,

    ads1299_command_rdatac = 0x10,
    ads1299_command_sdatac = 0x11,

    ads1299_command_rreg = 0x20,
    ads1299_command_wreg = 0x40
};

typedef struct ads1299_register_packet
{
    uint8_t id;

    uint8_t config1;
    uint8_t config2;
    uint8_t config3;

    uint8_t loff;

    uint8_t chnset[8];
    
    uint8_t bias_sensp;
    uint8_t bias_sensn;

    uint8_t loff_sensp;
    uint8_t loff_sensn;

    uint8_t loff_flip;

    uint8_t loff_statp;
    uint8_t loff_statn;

    uint8_t gpio;

    uint8_t misc1;
    uint8_t misc2;
    
    uint8_t config4;
} __attribute__ ((packed)) ads1299_register_packet;

typedef struct ads1299_data_packet
{
    uint32_t stat : 24;

    uint8_t channel_data[24];
} __attribute__ ((packed)) ads1299_data_packet;

typedef struct openbci_data_packet
{
    uint8_t header;

    uint8_t sample_number;

    uint8_t channel_data[24];

    uint8_t auxiliary_data[6];

    uint8_t footer;
} __attribute__ ((packed)) openbci_data_packet;

ads1299_register_packet ads1299_register_buffer = {};
ads1299_data_packet ads1299_data_buffer = {};

openbci_data_packet openbci_data_buffer[OPENBCI_DATA_BUFFER_SIZE] = {{}};

uint16_t openbci_data_buffer_head = 0;
uint16_t openbci_data_buffer_tail = 0;

uint8_t channel_setting_buffer[8] = {0};

uint8_t sample_counter = 0;

bool streaming_enabled = false;

uint8_t* tcp_transfer_buffer = NULL;

IPAddress local_ip(192, 168, 4, 1);
IPAddress network_gateway(192, 168, 4, 1);
IPAddress subnet_mask(255, 255, 255, 0);

WebServer web_server(80);
WiFiClient tcp_client;

size_t wifi_latency = 0;

void IRAM_ATTR ads1299_read_buffer(void* input_buffer, size_t buffer_size)
{
    spiTransferBytesNL(SPI.bus(), NULL, (uint8_t*)input_buffer, buffer_size);
}

void IRAM_ATTR ads1299_write_byte(uint8_t byte_to_write)
{
    spiWriteByteNL(SPI.bus(), byte_to_write);
}

void IRAM_ATTR ads1299_write_buffer(void* output_buffer, size_t buffer_size)
{
    spiWriteNL(SPI.bus(), (uint8_t*)output_buffer, buffer_size);
}

void ads1299_load_registers()
{
    ads1299_write_byte(ads1299_command_rreg);
    ads1299_write_byte(sizeof(ads1299_register_packet) - 1);

    ads1299_read_buffer(&ads1299_register_buffer, sizeof(ads1299_register_packet));
}

void ads1299_flush_registers()
{
    ads1299_write_byte(ads1299_command_wreg);
    ads1299_write_byte(sizeof(ads1299_register_packet) - 1);
  
    ads1299_write_buffer(&ads1299_register_buffer, sizeof(ads1299_register_packet));
}

void IRAM_ATTR ads1299_drdy_interrupt()
{
    ads1299_read_buffer(&ads1299_data_buffer, sizeof(ads1299_data_packet));
    
    if (streaming_enabled)
    {
        openbci_data_buffer[openbci_data_buffer_tail].header = 0xA0;
        openbci_data_buffer[openbci_data_buffer_tail].sample_number = sample_counter++;
      
        memcpy(&openbci_data_buffer[openbci_data_buffer_tail].channel_data, &ads1299_data_buffer.channel_data, sizeof(ads1299_data_buffer.channel_data));
     
        memset(&openbci_data_buffer[openbci_data_buffer_tail].auxiliary_data, 0x00, sizeof(openbci_data_buffer[openbci_data_buffer_tail].auxiliary_data));
      
        openbci_data_buffer[openbci_data_buffer_tail].footer = 0xC0;

        if (++openbci_data_buffer_tail >= 50) openbci_data_buffer_tail = 0;
    }
}

size_t get_sampling_rate()
{
    return 16000 >> (ads1299_register_buffer.config1 & 0b111);
}

size_t get_sample_delay()
{
    return 1000000 / get_sampling_rate();
}

size_t gain_from_channel(uint8_t channel_index)
{
    uint8_t gain = (ads1299_register_buffer.chnset[channel_index] >> 4) & 0b111;
    
    switch (gain)
    {
        case 0b000:
          return 1;
        case 0b001:
          return 2;
        case 0b010:
          return 4;
        case 0b011:
          return 6;
        case 0b100:
          return 8;
        case 0b101:
          return 12;
        case 0b110:
          return 24;
        default:
          return 0;
    }
}

IPAddress ip_from_string(String ip_string)
{
    IPAddress ip_address(0, 0, 0, 0);

    ip_address.fromString(ip_string);

    return ip_address;
}

uint8_t digit_from_char(char digit_char)
{
    return digit_char - '0';
}

void get_system_info()
{
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
    
    JsonObject json_object = json_document.to<JsonObject>();
  
    json_object["board_connected"] = true;
    json_object["heap"] = ESP.getFreeHeap();
    json_object["ip"] = WiFi.softAPIP().toString();
    json_object["latency"] = wifi_latency;
    json_object["mac"] = WiFi.softAPmacAddress();
    json_object["name"] = OPENBCI_NAME;
    json_object["num_channels"] = 8;
    json_object["version"] = OPENBCI_VERSION;
  
    String json_string = "";
  
    serializeJson(json_document, json_string);
    
    web_server.send(200, "text/json", json_string);
}

void get_board_info()
{
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
  
    JsonObject json_object = json_document.to<JsonObject>();
  
    json_object["board_connected"] = true;
    json_object["board_type"] = "cyton";
    
    JsonArray gains = json_object.createNestedArray("gains");
  
    for (size_t channel_index = 0; channel_index < 8; channel_index++) gains.add(gain_from_channel(channel_index));

    json_object["num_channels"] = 8;
  
    String json_string = "";
  
    serializeJson(json_document, json_string);
 
    web_server.send(200, "text/json", json_string);
}

void process_command()
{    
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
    
    deserializeJson(json_document, web_server.arg(0));
  
    JsonObject json_object = json_document.as<JsonObject>();

    String command = json_object["command"];

    String return_message = "";

    bool streaming_state = streaming_enabled;
    
    streaming_enabled = false;

    ads1299_write_byte(ads1299_command_sdatac);
  
    delayMicroseconds(50);
    
    if (command[0] == '~')
    {
        uint8_t sampling_rate = digit_from_char(command[1]);

        ads1299_register_buffer.config1 &= ~(0b111);
        ads1299_register_buffer.config1 |= sampling_rate;
        
        return_message = "Success: Sample rate is now ";
        return_message += get_sampling_rate();
        return_message += "Hz";
    }
    else if (command == "1") ads1299_register_buffer.chnset[0] = 0b10000001;
    else if (command == "2") ads1299_register_buffer.chnset[1] = 0b10000001;
    else if (command == "3") ads1299_register_buffer.chnset[2] = 0b10000001;
    else if (command == "4") ads1299_register_buffer.chnset[3] = 0b10000001;
    else if (command == "5") ads1299_register_buffer.chnset[4] = 0b10000001;
    else if (command == "6") ads1299_register_buffer.chnset[5] = 0b10000001;
    else if (command == "7") ads1299_register_buffer.chnset[6] = 0b10000001;
    else if (command == "8") ads1299_register_buffer.chnset[7] = 0b10000001;
    else if (command == "!") ads1299_register_buffer.chnset[0] = channel_setting_buffer[0];
    else if (command == "@") ads1299_register_buffer.chnset[1] = channel_setting_buffer[1];
    else if (command == "#") ads1299_register_buffer.chnset[2] = channel_setting_buffer[2];
    else if (command == "$") ads1299_register_buffer.chnset[3] = channel_setting_buffer[3];
    else if (command == "%") ads1299_register_buffer.chnset[4] = channel_setting_buffer[4];
    else if (command == "^") ads1299_register_buffer.chnset[5] = channel_setting_buffer[5];
    else if (command == "&") ads1299_register_buffer.chnset[6] = channel_setting_buffer[6];
    else if (command == "*") ads1299_register_buffer.chnset[7] = channel_setting_buffer[7];
    else if (command[0] == 'x')
    {
       uint8_t channel_index = digit_from_char(command[1]) - 1;

       uint8_t channel_power_down = digit_from_char(command[2]);
       uint8_t channel_gain = digit_from_char(command[3]);
       uint8_t channel_source = digit_from_char(command[4]);
       uint8_t channel_bias_enabled = digit_from_char(command[5]);
       uint8_t channel_srb2_enabled = digit_from_char(command[6]);
 
       uint8_t channel_setting = (channel_power_down << 7) | (channel_gain << 4) | (channel_srb2_enabled << 3) | channel_source;

       channel_setting_buffer[channel_index] = channel_setting;
       ads1299_register_buffer.chnset[channel_index] = channel_setting;

       ads1299_register_buffer.bias_sensp &= ~(1 << channel_index);
       ads1299_register_buffer.bias_sensp |= (channel_bias_enabled << channel_index);

       ads1299_register_buffer.bias_sensn &= ~(1 << channel_index);
       ads1299_register_buffer.bias_sensn |= (channel_bias_enabled << channel_index);

       uint8_t srb1_enabled = digit_from_char(command[7]);

       ads1299_register_buffer.misc1 &= ~(0b00100000);
       ads1299_register_buffer.misc1 |= (srb1_enabled << 5);
    }
    else if (command == "b") streaming_state = true;
    else if (command == "s") streaming_state = false;

    ads1299_flush_registers();
    
    ads1299_write_byte(ads1299_command_rdatac);
  
    delayMicroseconds(50);

    streaming_enabled = streaming_state;

    web_server.send(200, "text/plain", return_message);
}

void start_streaming()
{
    streaming_enabled = true;

    web_server.send(200);
}

void stop_streaming()
{
    streaming_enabled = false;

    web_server.send(200);
}

void switch_raw_output()
{
    web_server.send(200, "text/plain", "Output mode configured to raw");
}

void get_tcp_config()
{
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
  
    JsonObject json_object = json_document.to<JsonObject>();
    
    json_object["connected"] = (tcp_client.connected() != 0) ? true : false;
    json_object["delimiter"] = false;
    json_object["ip_address"] = tcp_client.remoteIP().toString();
    json_object["output"] = "raw";
    json_object["port"] = tcp_client.remotePort();
    json_object["latency"] = wifi_latency;
  
    String json_string = "";
  
    serializeJson(json_document, json_string);

    web_server.send(200, "text/json", json_string);
}

void set_tcp_config()
{
    streaming_enabled = false;
    
    DynamicJsonDocument json_document(JSON_BUFFER_SIZE);
    
    deserializeJson(json_document, web_server.arg(0));
  
    JsonObject json_object = json_document.as<JsonObject>();
  
    String tcp_client_ip = json_object["ip"];
    wifi_latency = json_object["latency"];
    uint16_t tcp_client_port = json_object["port"];
    
    tcp_client.stop();
    
    tcp_client.connect(ip_from_string(tcp_client_ip), tcp_client_port);

    tcp_client.setNoDelay(1);
    
    get_tcp_config();
}

void stop_tcp_connection()
{
    streaming_enabled = false;
    
    tcp_client.stop();

    get_tcp_config();
}

void invalid_request()
{
    web_server.send(404, "text/plain", "Invalid Request!");
}

void setup()
{ 
    pinMode(ADS1299_PIN_RESET, OUTPUT);
    digitalWrite(ADS1299_PIN_RESET, LOW);
  
    pinMode(ADS1299_PIN_DRDY, INPUT);
  
    delayMicroseconds(50);

    SPI.begin(ADS1299_PIN_SCK, ADS1299_PIN_MISO, ADS1299_PIN_MOSI, ADS1299_PIN_SS);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

    pinMode(ADS1299_PIN_SS, OUTPUT);
    digitalWrite(ADS1299_PIN_SS, LOW);
  
    delay(5);
  
    digitalWrite(ADS1299_PIN_RESET, HIGH);
    delay(25);
    digitalWrite(ADS1299_PIN_RESET, LOW);
    delay(25);
    digitalWrite(ADS1299_PIN_RESET, HIGH);
  
    delay(5);
  
    ads1299_write_byte(ads1299_command_sdatac);
    
    delayMicroseconds(50);
    
    ads1299_write_byte(ads1299_command_stop);
    
    delayMicroseconds(50);

    ads1299_load_registers();

    ads1299_register_buffer.config1 = 0b10010101;
    ads1299_register_buffer.config2 = 0b11010001;
    ads1299_register_buffer.config3 = 0b11111100;

    ads1299_register_buffer.loff = 0b00000000;
    
    for (uint8_t channel_index = 0; channel_index < 8; channel_index++) ads1299_register_buffer.chnset[channel_index] = 0b01101000;
    
    ads1299_register_buffer.bias_sensp = 0b11111111;
    ads1299_register_buffer.bias_sensn = 0b11111111;
    
    ads1299_register_buffer.loff_sensp = 0b00000000;
    ads1299_register_buffer.loff_sensn = 0b00000000;
    
    ads1299_register_buffer.loff_flip = 0b00000000;
  
    ads1299_register_buffer.gpio = 0b00001111;
  
    ads1299_register_buffer.misc1 = 0b00000000;
    ads1299_register_buffer.misc2 = 0b00000000;

    ads1299_register_buffer.config4 = 0b00000000;

    ads1299_flush_registers();
  
    delayMicroseconds(50);
  
    ads1299_write_byte(ads1299_command_start);
    
    delayMicroseconds(50);
    
    ads1299_write_byte(ads1299_command_rdatac);

    attachInterrupt(digitalPinToInterrupt(ADS1299_PIN_DRDY), ads1299_drdy_interrupt, FALLING);
    
    delayMicroseconds(50);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(SOFT_AP_SSID, SOFT_AP_PASSWORD);
      
    delay(250);
      
    WiFi.softAPConfig(local_ip, network_gateway, subnet_mask);
      
    delay(250);

    MDNS.begin("openbci");
    
    tcp_transfer_buffer = (uint8_t*)malloc(sizeof(openbci_data_buffer));

    web_server.on("/all", HTTP_GET, get_system_info);
    web_server.on("/board", HTTP_GET, get_board_info);

    web_server.on("/command", HTTP_POST, process_command);
      
    web_server.on("/steam/start", HTTP_GET, start_streaming);
    web_server.on("/steam/stop", HTTP_GET, start_streaming);

    web_server.on("/output/raw", HTTP_GET, switch_raw_output);
    
    web_server.on("/tcp", HTTP_GET, get_tcp_config);
    web_server.on("/tcp", HTTP_POST, set_tcp_config);
    web_server.on("/tcp", HTTP_DELETE, stop_tcp_connection);
    
    web_server.onNotFound(invalid_request);
      
    MDNS.addService("http", "tcp", 80);
      
    web_server.begin();
}

uint64_t last_micros = 0;

void loop()
{
    if (streaming_enabled == true)
    {
        uint64_t current_micros = micros();
      
        size_t tcp_write_size = wifi_latency / get_sample_delay();

        int16_t packets_to_write = openbci_data_buffer_tail - openbci_data_buffer_head;

        if (packets_to_write < 0) packets_to_write += OPENBCI_DATA_BUFFER_SIZE;
        
        if ((last_micros + wifi_latency <= current_micros) || (packets_to_write >= tcp_write_size))
        {              
            if (openbci_data_buffer_head + packets_to_write >= OPENBCI_DATA_BUFFER_SIZE)
            { 
               size_t wrap_size = OPENBCI_DATA_BUFFER_SIZE - openbci_data_buffer_head;

               memcpy(tcp_transfer_buffer, &openbci_data_buffer[openbci_data_buffer_head], wrap_size * sizeof(openbci_data_packet));
               memcpy(tcp_transfer_buffer + (wrap_size * sizeof(openbci_data_packet)), &openbci_data_buffer, (packets_to_write - wrap_size) * sizeof(openbci_data_packet));
            }

            else memcpy(tcp_transfer_buffer, &openbci_data_buffer[openbci_data_buffer_head], packets_to_write * sizeof(openbci_data_packet));
            
            tcp_client.write(tcp_transfer_buffer, packets_to_write * sizeof(openbci_data_packet));   

            openbci_data_buffer_head = openbci_data_buffer_tail;

            last_micros = current_micros;
        }
    }
    
    web_server.handleClient();
}
