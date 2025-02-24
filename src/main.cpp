#include <Arduino.h>
#include <ArduinoOTA.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <math.h>
#include <Wire.h>          // Include Wire for I2C
#include <si5351.h>        // Include Etherkit Si5351 library

// I2C configuration for Si5351
#define I2C_SDA_PIN 19  // Use GPIO 19 for SDA
#define I2C_SCL_PIN 18  // Use GPIO 18 for SCL

// I2S configuration
#define I2S_BCK_PIN 26
#define I2S_WS_PIN 25
#define I2S_DOUT_PIN 22
#define SAMPLE_RATE 44100
#define DEFAULT_FREQ 1000
#define AMPLITUDE 32767
#define BUFFER_SIZE 8192

// Si5351 clock frequencies
uint64_t clk0_freq = 100000000ULL; // 100 MHz
uint64_t clk1_freq = 50000000ULL;  // 50 MHz
uint64_t clk2_freq = 25000000ULL;  // 25 MHz

// Si5351 object
Si5351 si5351;

// Waveform types
enum Waveform {
  SINE,
  TRIANGLE,
  SAWTOOTH,
  SQUARE
};

// Web server
WebServer server(80);

// Waveform settings
volatile Waveform waveform = SINE;       // Default waveform
volatile float left_freq = DEFAULT_FREQ;  // Left channel frequency
volatile float right_freq = DEFAULT_FREQ; // Right channel frequency
volatile float phase_shift = 0.0;         // Phase shift between channels (in radians)

// Buffer for waveform (stereo: left and right channels)
int16_t waveform_buffer[BUFFER_SIZE * 2]; // Stereo buffer

// Current position in the buffer
volatile int buffer_pos = 0;

// Phase tracking for continuous waveform generation
volatile float left_phase = 0.0;
volatile float right_phase = 0.0;

// Function to generate waveform
void generateWaveform() {
  float left_phase_increment = 2 * M_PI * left_freq / SAMPLE_RATE;
  float right_phase_increment = 2 * M_PI * right_freq / SAMPLE_RATE;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    int16_t left_sample = 0;
    int16_t right_sample = 0;

    // Apply phase shift to the right channel
    float shifted_right_phase = right_phase + phase_shift;

    switch (waveform) {
      case SINE:
        left_sample = AMPLITUDE * sin(left_phase);
        right_sample = AMPLITUDE * sin(shifted_right_phase);
        break;
      case TRIANGLE:
        left_sample = AMPLITUDE * (2.0 * fabs(2.0 * (left_phase / (2 * M_PI) - floor(left_phase / (2 * M_PI) + 0.5))) - 1.0);
        right_sample = AMPLITUDE * (2.0 * fabs(2.0 * (shifted_right_phase / (2 * M_PI) - floor(shifted_right_phase / (2 * M_PI) + 0.5))) - 1.0);
        break;
      case SAWTOOTH:
        left_sample = AMPLITUDE * (2.0 * (left_phase / (2 * M_PI) - floor(left_phase / (2 * M_PI) + 0.5)));
        right_sample = AMPLITUDE * (2.0 * (shifted_right_phase / (2 * M_PI) - floor(shifted_right_phase / (2 * M_PI) + 0.5)));
        break;
      case SQUARE:
        left_sample = (sin(left_phase) >= 0) ? AMPLITUDE : -AMPLITUDE;
        right_sample = (sin(shifted_right_phase) >= 0) ? AMPLITUDE : -AMPLITUDE;
        break;
    }

    waveform_buffer[i * 2] = left_sample;     // Left channel
    waveform_buffer[i * 2 + 1] = right_sample; // Right channel

    // Increment phases
    left_phase += left_phase_increment;
    right_phase += right_phase_increment;

    // Wrap phases to avoid overflow
    if (left_phase >= 2 * M_PI) left_phase -= 2 * M_PI;
    if (right_phase >= 2 * M_PI) right_phase -= 2 * M_PI;
  }
}

// Handle waveform and parameter update
void handleSetWaveform() {
  if (server.hasArg("waveform") && server.hasArg("left_freq") && server.hasArg("right_freq") && server.hasArg("phase_shift")) {
    int new_waveform = server.arg("waveform").toInt();
    float new_left_freq = server.arg("left_freq").toFloat();
    float new_right_freq = server.arg("right_freq").toFloat();
    float new_phase_shift = server.arg("phase_shift").toFloat() * M_PI / 180; // Convert degrees to radians

    if (new_waveform >= 0 && new_waveform <= 3 &&
        new_left_freq >= 1 && new_left_freq <= 20000 &&
        new_right_freq >= 1 && new_right_freq <= 20000 &&
        new_phase_shift >= 0 && new_phase_shift <= 2 * M_PI) {
      waveform = static_cast<Waveform>(new_waveform);
      left_freq = new_left_freq;
      right_freq = new_right_freq;
      phase_shift = new_phase_shift;
      generateWaveform(); // Regenerate waveform with new settings
      buffer_pos = 0;     // Reset buffer position
    }
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle equalizing frequencies
void handleEqualizeFreq() {
  right_freq = left_freq; // Set right frequency equal to left frequency
  generateWaveform();     // Regenerate waveform with updated frequencies
  buffer_pos = 0;         // Reset buffer position
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle root URL
void handleRoot() {
  String html = R"(
    <html>
      <body>
        <h1>ESP32 Waveform Generator</h1>
        <p>Connected to: )" + String(WiFi.SSID()) + R"(</p>
        <p>IP Address: )" + WiFi.localIP().toString() + R"(</p>
        <form action="/setwaveform" method="GET">
          <label for="waveform">Waveform:</label>
          <select id="waveform" name="waveform">
            <option value="0")" + (waveform == SINE ? " selected" : "") + R"(>Sine</option>
            <option value="1")" + (waveform == TRIANGLE ? " selected" : "") + R"(>Triangle</option>
            <option value="2")" + (waveform == SAWTOOTH ? " selected" : "") + R"(>Sawtooth</option>
            <option value="3")" + (waveform == SQUARE ? " selected" : "") + R"(>Square</option>
          </select>
          <br>
          <label for="left_freq">Left Channel Frequency (Hz):</label>
          <input type="number" id="left_freq" name="left_freq" min="1" max="20000" value=")" + String(left_freq) + R"(">
          <br>
          <label for="right_freq">Right Channel Frequency (Hz):</label>
          <input type="number" id="right_freq" name="right_freq" min="1" max="20000" value=")" + String(right_freq) + R"(">
          <br>
          <label for="phase_shift">Phase Shift (degrees):</label>
          <input type="number" id="phase_shift" name="phase_shift" min="0" max="360" value=")" + String(phase_shift * 180 / M_PI) + R"(">
          <br>
          <input type="submit" value="Set Waveform and Parameters">
        </form>
        <form action="/equalizefreq" method="GET">
          <input type="submit" value="Make Frequencies Equal">
        </form>
        <form action="/setsi5351" method="GET">
          <label for="clk0_freq">Si5351 CLK0 Frequency (Hz):</label>
          <input type="number" id="clk0_freq" name="clk0_freq" min="100000" max="200000000" value=")" + String(clk0_freq) + R"(">
          <br>
          <label for="clk1_freq">Si5351 CLK1 Frequency (Hz):</label>
          <input type="number" id="clk1_freq" name="clk1_freq" min="100000" max="200000000" value=")" + String(clk1_freq) + R"(">
          <br>
          <label for="clk2_freq">Si5351 CLK2 Frequency (Hz):</label>
          <input type="number" id="clk2_freq" name="clk2_freq" min="100000" max="200000000" value=")" + String(clk2_freq) + R"(">
          <br>
          <input type="submit" value="Set Si5351 Frequencies">
        </form>
      </body>
    </html>
  )";
  server.send(200, "text/html", html);
}

// Handle Si5351 frequency update
void handleSetSi5351() {
  if (server.hasArg("clk0_freq") && server.hasArg("clk1_freq") && server.hasArg("clk2_freq")) {
    clk0_freq = server.arg("clk0_freq").toInt(); // Input is already in Hz
    clk1_freq = server.arg("clk1_freq").toInt(); // Input is already in Hz
    clk2_freq = server.arg("clk2_freq").toInt(); // Input is already in Hz

    si5351.set_freq(clk0_freq, SI5351_CLK0);
    si5351.set_freq(clk1_freq, SI5351_CLK1);
    si5351.set_freq(clk2_freq, SI5351_CLK2);
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

// Initialize Si5351
void setupSi5351() {
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if (!i2c_found) {
    Serial.println("Si5351 not found on I2C bus! Check wiring and I2C address.");
    while (1); // Halt if initialization fails
  }

  // Set initial frequencies using global variables
  si5351.set_freq(clk0_freq, SI5351_CLK0);
  si5351.set_freq(clk1_freq, SI5351_CLK1);
  si5351.set_freq(clk2_freq, SI5351_CLK2);

  // Enable outputs
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK1, 1);
  si5351.output_enable(SI5351_CLK2, 1);

  Serial.println("Si5351 initialized successfully!");
}
void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Initialize I2C with custom pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Initialize Si5351
  setupSi5351();

  // Initialize WiFiManager
  WiFiManager wifiManager;

  // Attempt to connect to WiFi
  if (!wifiManager.autoConnect("ESP32-Waveform")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    ESP.restart(); // Restart ESP32 if connection fails
  }

  // Print connection details
  Serial.println("WiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 16,
    .dma_buf_len = 512,
    .use_apll = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_DOUT_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // Generate initial waveform
  generateWaveform();

  // Start web server
  server.on("/", handleRoot);
  server.on("/setwaveform", handleSetWaveform);
  server.on("/equalizefreq", handleEqualizeFreq);
  server.on("/setsi5351", handleSetSi5351);
  server.begin();
  Serial.println("HTTP server started");

  // OTA setup
  ArduinoOTA.begin();
}

void loop() {
  // Handle web server requests
  server.handleClient();

  // Handle OTA updates
  ArduinoOTA.handle();

  // Stream the waveform to the DAC in smaller chunks
  size_t bytes_written;
  int chunk_size = 128; // Number of samples to stream at a time
  for (int i = 0; i < chunk_size; i++) {
    i2s_write(I2S_NUM_0, &waveform_buffer[buffer_pos * 2], sizeof(int16_t) * 2, &bytes_written, portMAX_DELAY);
    buffer_pos = (buffer_pos + 1) % BUFFER_SIZE; // Circular buffer

    // Regenerate waveform if we reach the end of the buffer
    if (buffer_pos == 0) {
      generateWaveform();
    }
  }
}