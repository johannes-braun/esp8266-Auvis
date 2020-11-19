#define FFT_SPEED_OVER_PRECISION
#define FFT_SQRT_APPROXIMATION

#include <Arduino.h>
#include <user_interface.h>
#include <array>
#include <arduinoFFT.h>
#include <FastLED.h>
#include <FS.h>
#include <WString.h>
#include <sstream>
#include <LittleFS.h>
#include "util.hpp"

#include "site_raw.hpp"
#include "server.hpp"
#include "8x8_font.h"

using fft_float = float;

namespace defaults {
  constexpr size_t num_bins = 64;
  constexpr double adc_scale = 1.0 / 1024.0;
}

ADC_MODE(ADC_TOUT)

CRGB operator*(CRGB const& a, double v)
{
  return CRGB(a.r * v, a.g * v, a.b * v);
}
float mix(float a, float b, float t)
{
  return (1 - t) * a + t * b;
}
CRGB mixrgb(CRGB a, CRGB b, float t)
{
  return CRGB(mix(a.r, b.r, t), mix(a.g, b.g, t), mix(a.b, b.b, t));
}

enum class arg_type
{
  integer, floating_point, boolean
};

template<typename T> struct deduce_arg_type;
template<> struct deduce_arg_type<char> { constexpr static arg_type arg = arg_type::integer; };
template<> struct deduce_arg_type<short> { constexpr static arg_type arg = arg_type::integer; };
template<> struct deduce_arg_type<int> { constexpr static arg_type arg = arg_type::integer; };
template<> struct deduce_arg_type<unsigned int> { constexpr static arg_type arg = arg_type::integer; };
template<> struct deduce_arg_type<unsigned short> { constexpr static arg_type arg = arg_type::integer; };
template<> struct deduce_arg_type<unsigned char> { constexpr static arg_type arg = arg_type::integer; };
template<> struct deduce_arg_type<float> { constexpr static arg_type arg = arg_type::floating_point; };
template<> struct deduce_arg_type<double> { constexpr static arg_type arg = arg_type::floating_point; };
template<> struct deduce_arg_type<bool> { constexpr static arg_type arg = arg_type::boolean; };

template<arg_type T> struct arg_type_to_type;
template<> struct arg_type_to_type<arg_type::integer> { using type = int; };
template<> struct arg_type_to_type<arg_type::floating_point> { using type = double; };
template<> struct arg_type_to_type<arg_type::boolean> { using type = bool; };

template<arg_type Cvt> inline __attribute__((always_inline)) void string_to_impl(const String& str, int& value) { value = str.toInt(); }
template<arg_type Cvt> inline __attribute__((always_inline)) void string_to_impl(const String& str, double& value) { value = str.toDouble(); }
template<arg_type Cvt> inline __attribute__((always_inline)) void string_to_impl(const String& str, bool& value) { value = str == "true" || str.toInt() != 0; }

template<arg_type Cvt>
auto string_to(const String& str) -> typename arg_type_to_type<Cvt>::type
{
  typename arg_type_to_type<Cvt>::type result {};
  string_to_impl<Cvt>(str, result);
  return result;
}

template<typename T, arg_type Cvt = deduce_arg_type<T>::arg>
class server_interface
{
public:
  template<typename V, typename std::enable_if<std::is_convertible<V, T>::value>::type* = nullptr>
  server_interface(String name, V&& initial_value)
    : m_name(name), m_value(std::forward<V>(initial_value))
  {

  }

  bool load_if_arg(const ESP8266WebServer& server)
  {
    if(server.hasArg(m_name))
    {
      m_value = string_to<Cvt>(server.arg(m_name));
      return true;
    }
    return false;
  }

  T const& operator*() const noexcept {
    return m_value;
  }
  T& operator*() noexcept {
    return m_value;
  }

private:
  String m_name;
  T m_value;
};

struct state_t {
  const uint16 calibration_version = 1000;

  std::vector<fft_float> audio_values;
  std::vector<fft_float> audio_imag;
  std::vector<fft_float> audio_presentation;
  std::vector<fft_float> fft_baselines;
  std::vector<fft_float> fft_tops;

  bool baseline_active = false;
  bool tops_active = false;
  
  uint64 last_loop_micros;
  uint64 last_micros;

  std::array<fft_float, 8> binned_interpolated;
  std::array<fft_float, 8> binned_presentation;

  void (*display_loop)(double deltas) = nullptr;

  std::array<CRGB, 64> rgbs;
  auvis::Server web_server;

  struct {
    uint64 last_loop_micros_overflows = 0;
    uint64 last_micros_overflows = 0;
  } statistics;

  struct {
    const uint16 version = 1002;

    server_interface<bool> calibrate_base{ "calibrate_base", false };
    server_interface<bool> calibrate_tops{ "calibrate_tops", false };
    server_interface<fft_float> baseline_offset_exponent{ "baseline_offset_exponent", 0.95 };
    server_interface<fft_float> fall_duration{ "fall_duration", 0.2 };
    server_interface<float> brightness{ "brightness", 0.1 };
    server_interface<fft_float> audio_scale_exponent{ "audio_scale_exponent", 0.6f };
    server_interface<CRGB, arg_type::integer> upper_end{ "upper_end", CRGB(255, 20, 0) };
    server_interface<CRGB, arg_type::integer> lower_end{ "lower_end", CRGB(100, 100, 100) };
    server_interface<fft_float> post_average_baseline{ "post_average_baseline", 0.0f };
    server_interface<unsigned> sample_frequency{ "sample_frequency", 5000 };
  } config;
} state;

std::string create_web_page();

template<typename T, typename Writable>
void write_value(Writable& writable, T const& value)
{
  writable.write(reinterpret_cast<const char*>(&value), sizeof(T));
}
template<typename Writable, typename T, arg_type Cvt>
void write_interface(Writable& writable, server_interface<T, Cvt> const& value)
{
  write_value(writable, *value);
}
template<typename Readable, typename T>
void read_value(Readable& readable, T& value)
{
  readable.readBytes(reinterpret_cast<char*>(&value), sizeof(T));
}
template<typename Readable, typename T, arg_type Cvt>
void read_interface(Readable& readable, server_interface<T, Cvt>& value)
{
  read_value(readable, *value);
}

template<typename T>
void read_vector(File& file, std::vector<T>& out, size_t desired_elements)
{
  out.resize(desired_elements);
  size_t actual_bytes = 0;
  read_value(file, actual_bytes);
  file.readBytes(reinterpret_cast<char*>(out.data()), actual_bytes);
}

template<typename T>
void write_vector(File& file, const std::vector<T>& in)
{
  const size_t bytes = in.size() * sizeof(T);
  write_value(file, bytes);
  file.write(reinterpret_cast<const char*>(in.data()), bytes);
}

void serialize_config() {
  auto f = LittleFS.open("/config.bin", "w");
  write_value(f, state.config.version);
  write_interface(f, state.config.baseline_offset_exponent);
  write_interface(f, state.config.fall_duration);
  write_interface(f, state.config.brightness);
  write_interface(f, state.config.audio_scale_exponent);
  write_interface(f, state.config.sample_frequency);
  write_interface(f, state.config.upper_end);
  write_interface(f, state.config.lower_end);
  write_interface(f, state.config.post_average_baseline);
  f.close();
}

void deserialize_config() {
  if(LittleFS.exists("/config.bin"))
  {
    auto f = LittleFS.open("/config.bin", "r");
    uint16 version;
    read_value(f, version);

    if(version >= 1001)
    {
      read_interface(f, state.config.baseline_offset_exponent);
      read_interface(f, state.config.fall_duration);
      read_interface(f, state.config.brightness);
      read_interface(f, state.config.audio_scale_exponent);
      read_interface(f, state.config.sample_frequency);
      read_interface(f, state.config.upper_end);
      read_interface(f, state.config.lower_end);
    }

    if(version >= 1002)
    {
      read_interface(f, state.config.post_average_baseline);
    }
    f.close();
  }
}

void write_data() {
  auto f = LittleFS.open("/calibration.bin", "w");
  write_value(f, state.calibration_version);
  write_vector(f, state.fft_baselines);
  write_vector(f, state.fft_tops);
  f.close();
}

void load_data() {
  if(LittleFS.exists("/calibration.bin"))
  {
    auto f = LittleFS.open("/calibration.bin", "r");
    uint16 version = 0;
    read_value(f, version);
    read_vector(f, state.fft_baselines, defaults::num_bins);
    read_vector(f, state.fft_tops, defaults::num_bins);
    f.close();
  }
}

void fft_loop(double delta);
void okay_loop_once(double delta);

void setup()
{
  state.audio_values.reserve(defaults::num_bins);
  state.audio_imag.reserve(defaults::num_bins);
  state.audio_presentation.resize(defaults::num_bins, 0.0);
  state.fft_baselines.resize(defaults::num_bins, 0.0);
  state.fft_tops.resize(defaults::num_bins, 0.0);

  state.statistics.last_loop_micros_overflows = 0;
  state.statistics.last_micros_overflows = 0;
  
  Serial.begin(9200);
  LittleFS.begin();
  load_data();
  deserialize_config();
  Serial.println("Loading data completed.");

  state.web_server.connect();
  state.web_server.on("/set", [](ESP8266WebServer& server){
    if(server.hasArg("reset_state"))
    {
      deserialize_config();
      load_data();
      server.send(200, "text/html", "State reset.");
    }
    else
    {
      state.config.calibrate_base.load_if_arg(server);
      state.config.calibrate_tops.load_if_arg(server);
      state.config.baseline_offset_exponent.load_if_arg(server);
      state.config.post_average_baseline.load_if_arg(server);
      state.config.fall_duration.load_if_arg(server);
      if(state.config.brightness.load_if_arg(server))
        FastLED.setBrightness(static_cast<uint8>(*state.config.brightness * 255.0));
      state.config.audio_scale_exponent.load_if_arg(server);
      state.config.lower_end.load_if_arg(server);
      state.config.upper_end.load_if_arg(server);
      state.config.sample_frequency.load_if_arg(server);

      if(server.hasArg("write_current_state"))
      {
        serialize_config();
        write_data();
        server.send(200, "text/html", "State written.");
      }
      else
      {
        server.send(200, "text/html", "done. Send 'write_current_state' to make your changes persistent.");
      }
    }
    state.display_loop = &okay_loop_once;
  });

  state.web_server.on("/status.json", [](ESP8266WebServer& server){
    std::stringstream status_stream;

    status_stream << "{"
      << "\"audio_values\": { \"size\": " << state.audio_values.size() << ", \"capacity\": " << state.audio_values.capacity() << " }, "
      << "\"audio_imag\": { \"size\": " << state.audio_imag.size() << ", \"capacity\": " << state.audio_imag.capacity() << " }, "
      << "\"audio_presentation\": { \"size\": " << state.audio_presentation.size() << ", \"capacity\": " << state.audio_presentation.capacity() << " }, "
      << "\"fft_baselines\": { \"size\": " << state.fft_baselines.size() << ", \"capacity\": " << state.fft_baselines.capacity() << " }, "
      << "\"fft_tops\": { \"size\": " << state.fft_tops.size() << ", \"capacity\": " << state.fft_tops.capacity() << " }, "
      << "\"baseline_active\": " << state.baseline_active << ", "
      << "\"tops_active\": " << state.baseline_active << ","
      << "\"last_loop_micros\": " << state.last_loop_micros << ","
      << "\"last_micros\": " << state.last_micros << ", "
      << "\"statistics\": { " 
      << "\"last_loop_micros_overflows\": " << state.statistics.last_loop_micros_overflows << ", "
      << "\"last_micros_overflows\": " << state.statistics.last_micros_overflows << "}, ";


    status_stream << "\"binned_interpolated\": [" << state.binned_interpolated[0];
    for(size_t i = 1; i < state.binned_interpolated.size(); ++i)
      status_stream << ", " << state.binned_interpolated[i];
    status_stream << "], ";
    status_stream << "\"binned_presentation\": [" << state.binned_presentation[0];
    for(size_t i = 1; i < state.binned_presentation.size(); ++i)
      status_stream << ", " << state.binned_presentation[i];
    status_stream << "]";

    status_stream << '}';

    server.send(200, "text/html", status_stream.str().c_str());
  });

  state.web_server.on("/", [](ESP8266WebServer& server) {
    server.send(200, "text/html", create_web_page().c_str());
  });

  Serial.println("Set callbacks");
  state.web_server.listen(80);

  pinMode(A0, INPUT);
  pinMode(D5, OUTPUT);

  state.last_micros = micros64();

  system_update_cpu_freq(SYS_CPU_160MHZ);

  FastLED.addLeds<NEOPIXEL, D5>(state.rgbs.data(), 64);
  FastLED.setBrightness(static_cast<uint8>(*state.config.brightness * 255.0));
  state.last_loop_micros = micros64();
  Serial.println("Setup done");
}

void do_avg_binning() {
  for(int i=0; i<8; ++i)
  {
    fft_float& avg = state.binned_presentation[i];
    avg = 0.0;
    for(int j = 0; j < defaults::num_bins/16; ++j)
    {
      const int index = i * defaults::num_bins/16 + j;
      const fft_float value = state.audio_presentation[index];
      avg += value;
    }
    avg /= defaults::num_bins/16;
    avg = std::pow(avg, *state.config.audio_scale_exponent);
    avg = auvis::smoothstep(avg, *state.config.post_average_baseline, 1.0f);
    avg = std::max<fft_float>(0.0, std::min<fft_float>(8.0, 9.0*avg));
  }
}

void loop()
{
  uint64 current = micros64();
  const unsigned micros_between_sample = unsigned(1000000.0 / *state.config.sample_frequency);

  if(current - state.last_micros > micros_between_sample) {
    if(state.audio_values.size() < defaults::num_bins)
    {
      state.audio_values.push_back(system_adc_read() * defaults::adc_scale);
      state.audio_imag.push_back(0.0);
    }
    else 
    {
      ArduinoFFT<fft_float> fft(state.audio_values.data(), state.audio_imag.data(), defaults::num_bins, *state.config.sample_frequency);
      fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      fft.compute(FFTDirection::Forward);
      fft.complexToMagnitude();

      if(*state.config.calibrate_base)
      {
        if(!state.baseline_active)
          for(auto& v : state.fft_baselines) v = std::numeric_limits<fft_float>::lowest();

        for(size_t i=0; i < state.audio_values.size(); ++i)
        {
          state.fft_baselines[i] = std::max(state.fft_baselines[i], std::pow(state.audio_values[i], *state.config.baseline_offset_exponent));
        }
        state.baseline_active = true;
      }
      else
      {
        if(state.baseline_active)
        {
          state.baseline_active = false;
        }
      }
      if(*state.config.calibrate_tops)
      {
        if(!state.tops_active)
          for(auto& v : state.fft_tops) v = std::numeric_limits<fft_float>::lowest();

        for(size_t i=0; i < state.audio_values.size(); ++i)
        {
          state.fft_tops[i] = std::max(state.fft_tops[i], state.audio_values[i]);
        }
        state.tops_active = true;
      }
      else
      {
        if(state.tops_active)
        {
          state.tops_active = false;
        }
      }

      for(size_t i=0; i < state.audio_values.size(); ++i)
      {
        state.audio_presentation[i] = std::max(fft_float(0), (state.audio_values[i] - state.fft_baselines[i]) / (state.fft_tops[i] - state.fft_baselines[i]));
      }

      do_avg_binning();

      state.audio_values.clear();
      state.audio_imag.clear();
    }

    if(current < state.last_micros)
      state.statistics.last_micros_overflows++;
    state.last_micros = current;
  }

  double delta_seconds = (current - state.last_loop_micros) / 1000000.0;
  if(delta_seconds >= 0.016)
  {
    if(state.display_loop)
      state.display_loop(delta_seconds);
    else
      fft_loop(delta_seconds);

    if(current < state.last_loop_micros)
      state.statistics.last_loop_micros_overflows++;
    state.last_loop_micros = current;
  }
  state.web_server.poll();
}

void fft_loop(double delta)
{
  for(int i=0; i < state.binned_interpolated.size(); ++i)
    {
      state.binned_interpolated[i] = std::max(fft_float(0), std::max(state.binned_interpolated[i] - 8.0f * static_cast<float>(delta) / (*state.config.fall_duration), state.binned_presentation[i]));
    }

    for(int x=0; x < state.binned_interpolated.size(); ++x)
    {
      for(int y = 0; y < 8; ++y)
      {
        const auto col = mixrgb(*state.config.lower_end, *state.config.upper_end, y / 7.0);

        if(state.binned_interpolated[7-x] > y)
        {
          state.rgbs[y * 8 + x] = col;
        }
        else
        {
          double frac = 1.0 - (state.binned_interpolated[7-x] - y);
          if(frac > 0.0 && frac < 1.0)
          {
            state.rgbs[y * 8 + x] = col * frac;
          }
          else
          {
            state.rgbs[y * 8 + x] = 0x0;
          }
        }
      }
    }
    FastLED.show();
}

void led_write_monochrome(uint8_t (&arr)[8], CRGB color)
{
  for(int x=0; x<8; ++x)
  {
    for(int y = 0; y < 8; ++y)
    {
      state.rgbs[y * 8 + x] = (arr[7 - y]>>x)&0b1 ? color : CRGB(0);
    }
  }
  FastLED.show();
}

void okay_loop_once(double delta)
{
  unsigned long letter_delay_ms = 250;
  led_write_monochrome(char_O, CRGB(120, 255, 120));
  delay(letter_delay_ms);
  led_write_monochrome(char_K, CRGB(120, 255, 120));
  delay(letter_delay_ms);
  state.display_loop = nullptr;
}

void hexvalue(std::stringstream& str, uint8 ch)
{
  char chars[3]{0, 0, 0};
  std::sprintf(chars, "%02X", ch);
  str << chars;
}

void jsvalue(std::stringstream& str, CRGB rgb)
{
  str << "'#";
  hexvalue(str, rgb.r);
  hexvalue(str, rgb.g);
  hexvalue(str, rgb.b);
  str << "'";
}

template<typename T>
void jsvalue(std::stringstream& str, T val)
{
  str << val;
}

std::string create_web_page()
{
  std::stringstream stream;
  stream << "<html><head><script>";

#define print_default(Name) do { stream << "const " #Name "_default="; jsvalue(stream, *state.config.Name); stream<<";";} while(0)

  print_default(calibrate_base);
  print_default(calibrate_tops);
  print_default(baseline_offset_exponent);
  print_default(fall_duration);
  print_default(brightness);
  print_default(audio_scale_exponent);
  print_default(upper_end);
  print_default(lower_end);
  print_default(sample_frequency);
  print_default(post_average_baseline);

#undef print_default
  stream << R"html(
</script>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="preconnect" href="https://fonts.gstatic.com">
  <link href="https://fonts.googleapis.com/css2?family=Montserrat:wght@300;500;600&display=swap" rel="stylesheet">
  <script src="https://cdn.jsdelivr.net/npm/@jaames/iro/dist/iro.min.js"></script>
  <style>
      #colors {
          display: flex;
          align-items: center;
          align-content: center;
          box-sizing: border-box;
      }
      #colors > * {
          padding: 4px;
      }

      button {
          padding: 8px 16px;
          width: 100%;
      }

      #root {
          display: flex;
          flex-direction: column;
          align-content: center;
          align-items: center;
          max-width: 500px;
          background: white;
          border-radius: 4px;
          margin: auto;
          padding: 10px;
          box-shadow: 0 0 40px #0003;
      }

      body {
          background: #eee;
          overflow: auto;
          font-family: 'Montserrat', sans-serif;
      }

      button {
        background: #1b76ed;
        color: white;
        border: none;
        outline: none;
        border-radius: 2px;
        cursor: pointer;
        font-family: inherit;
        margin: 2px;
      }
      button:hover {
        background: #3a8af3;
      }
      button:active {
        background: #2f49dd;
      }

      #root div {
        width: 100%;
        display: flex;
        align-items: flex-start;
        align-content: center;
        margin: 2px;
      }
      #root div label {
        width: 100%;
      }
      #root div input {
        width: 120px;
        box-sizing: border-box;
      }

      #root div input[type=number] {
        border: none;
        background: #ddd;
        padding: 4px 8px;
        border-radius: 2px;
      }

      #root div input[type=color] {
        border: none;
        background: #ddd;
        padding: 0;
        padding: 0px 8px;
      }
  </style>
</head>
<body>
    <script>
      const send = (val) => {
        var req = new XMLHttpRequest();
        req.open("GET", `/set?${val}`);
        req.send();
      }
      function hex_color_to_int(rrggbb) {
          return parseInt(rrggbb.substr(1, 6), 16);
      }

      const param_any = (id)=> `${id}=${document.getElementById(id).value}`
      const param_bool = (id)=> `${id}=${document.getElementById(id).checked}`
      const param_color = (id)=> `${id}=${hex_color_to_int(document.getElementById(id).value)}`

      const update_any = (el)=>send(param_any(el.id))
      const update_bool = (el)=>send(param_bool(el.id))
      const update_color = (el)=>send(param_color(el.id))
    </script>

    <div id="root">
      <h2>Calibration</h2>
      <div><label for="calibrate_base">Calibrate Baselines</label><input id="calibrate_base" type="checkbox"/></div>
      <div><label for="calibrate_tops">Calibrate Tops</label><input id="calibrate_tops" type="checkbox"/></div>
      <div><label for="baseline_offset_exponent">Calibration baseline exponent</label><input type="number" id="baseline_offset_exponent" min="0.0" step="0.05" max="10.0"/></div>

      <br/>
      
      <h2>Audio</h2>
      <div><label for="sample_frequency">Sample frequency</label><input type="number" id="sample_frequency" min="1000" step="500" max="10000"/></div>
      <div><label for="audio_scale_exponent">Audio scale exponent</label><input type="number" id="audio_scale_exponent" min="0.0" step="0.05" max="10.0"/></div>
      <div><label for="post_average_baseline">Post-Averaging Baseline</label><input type="number" id="post_average_baseline" min="0.0" step="0.01" max="1.0"/></div>
      
      <br/>
      
      <h2>Display</h2>
      <div><label for="fall_duration">Fall duration (s)</label><input type="number" id="fall_duration" min="0.01" step="0.01" max="10.0"/></div>
      <div><label for="brightness">Brightness</label><input type="number" id="brightness" min="0.0" step="0.05" max="1.0"/></div>

      <div><label for="upper_end">Upper end color</label><input type="color" id="upper_end"></div>
      <div><label for="lower_end">Lower end color</label><input type="color" id="lower_end"></div>

      <br/>

      <button id="button_persist">Persist current state</button>
      <button id="button_reset">Reset state</button>
  </div>

  <script>
    document.getElementById("calibrate_base").checked = calibrate_base_default;
    document.getElementById("calibrate_tops").checked = calibrate_tops_default;
    document.getElementById("baseline_offset_exponent").value = baseline_offset_exponent_default;
    document.getElementById("fall_duration").value = fall_duration_default;
    document.getElementById("brightness").value = brightness_default;
    document.getElementById("audio_scale_exponent").value = audio_scale_exponent_default;
    document.getElementById("upper_end").value = upper_end_default;
    document.getElementById("lower_end").value = lower_end_default;
    document.getElementById("sample_frequency").value = sample_frequency_default;
    document.getElementById("post_average_baseline").value = post_average_baseline_default;
    
    var checkboxes = document.getElementById("root").querySelectorAll('input[type=checkbox]');
    for(var i=0; i<checkboxes.length; ++i)
      checkboxes[i].setAttribute('onchange', 'update_bool(this)');
      
    var colorboxes = document.getElementById("root").querySelectorAll('input[type=color]');
    for(var i=0; i<colorboxes.length; ++i)
      colorboxes[i].setAttribute('onchange', 'update_color(this)');
      
    var numberboxes = document.getElementById("root").querySelectorAll('input[type=number]');
    for(var i=0; i<numberboxes.length; ++i)
      numberboxes[i].setAttribute('onchange', 'update_any(this)');

    document.getElementById("button_persist").onclick = ()=>{
        send("write_current_state");
    }
    
    document.getElementById("button_reset").onclick = ()=>{
        send("reset_state");
    }

  </script>
</body>
</html>
)html"; 
  return stream.str();
}