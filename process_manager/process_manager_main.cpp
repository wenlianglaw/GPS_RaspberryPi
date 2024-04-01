// Make sure there is one gps_raspberry_pi process running.
//
// This program checks the processs nameed "gps_raspberry_pi" every few seconds,
// if gps_raspberry_pi is not run, it starts "../gps_raspberry_pi /dev/ttyUSB1"
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <wiringPi.h>

namespace chrono = std::chrono;

constexpr const char *const kCmd = "pidof gps_raspberry_pi";

// https://stackoverflow.com/questions/52164723/how-to-execute-a-command-and-get-return-code-stdout-and-stderr-of-command-in-c
std::string exec(const char *cmd) {
  std::array<char, 128> buffer;
  std::string result;

  auto pipe = popen(cmd, "r"); // get rid of shared_ptr

  if (!pipe)
    throw std::runtime_error("popen() failed!");

  while (!feof(pipe)) {
    if (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
      result += buffer.data();
  }

  auto rc = pclose(pipe);

  if (rc == EXIT_SUCCESS) { // == 0

  } else if (rc == EXIT_FAILURE) { // EXIT_FAILURE is not used by all programs,
                                   // maybe needs some adaptation.
  }
  return result;
}

bool IsGpsRunning() {
  std::string cmd_output = exec(kCmd);
  return !cmd_output.empty();
}


void RunGPSMain() { system("/home/pi/programs/GPS_RaspberryPi/gps_raspberry_pi /dev/ttyUSB0"); }

// LED GPIO
constexpr int RED = 14;
constexpr int GREEN = 15;

int main() {

  wiringPiSetupGpio();
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);

  using namespace std::chrono_literals;

  std::thread run_gps_main;

  while (true) {
    bool is_gps_main_running = IsGpsRunning();
    if (!is_gps_main_running) {
      run_gps_main = std::thread(RunGPSMain);
      run_gps_main.detach();
      std::this_thread::sleep_for(2s);
    }

    std::cout << "Progress is running: " << is_gps_main_running << std::endl;

    // Turn on light for status
    if (is_gps_main_running) {
      for (int i = 0; i < 2; i++) {
        digitalWrite(GREEN, HIGH);
        std::this_thread::sleep_for(.1s);
        digitalWrite(GREEN, LOW);
        std::this_thread::sleep_for(.2s);
      }
      digitalWrite(GREEN, LOW);
    } else {
      for (int i = 0; i < 2; i++) {
        digitalWrite(RED, HIGH);
        std::this_thread::sleep_for(.1s);
        digitalWrite(RED, LOW);
        std::this_thread::sleep_for(.1s);
      }
      digitalWrite(RED, LOW);
    }

    std::this_thread::sleep_for(2s);
  }

  return 0;
}
