// This program ensures that gps_raspberry_pi process is always in running and
// lightening the LED light to indicate the running status.  If LED light is not
// on.  You need to restart system to run this program again (required setting
// cron job)
//
// This program checks to see if there are two processes nameed "gps_raspberry_pi"
// every few seconds.  If there are less than 2 processes, this program kills the
// current running process and starts the following processes.
//   - "/home/pi/programs/GPS_RaspberryPi/gps_raspberry_pi /dev/ttyUSB0"
//   - "/home/pi/programs/GPS_RaspberryPi/gps_raspberry_pi /dev/ttyUSB1"
#include <chrono>
#include <iostream>
#include <sstream>
#include <string.h>
#include <thread>
#include <unistd.h>

#include <wiringPi.h>

namespace chrono = std::chrono;

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

void GetRunningPid(int *a, int *b) {
  std::string cmd_output = exec("pidof gps_raspberry_pi");
  std::istringstream ss(cmd_output);
  if (!ss.eof())
    ss >> *a;
  if (!ss.eof())
    ss >> *b;

  std::cout << "PID: " << *a << " " << *b;
}

void RunGPSMain0() {
  system("/home/pi/programs/GPS_RaspberryPi/gps_raspberry_pi /dev/ttyUSB0");
}
void RunGPSMain1() {
  system("/home/pi/programs/GPS_RaspberryPi/gps_raspberry_pi /dev/ttyUSB1");
}

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
    int pid1 = -1, pid2 = -1;
    GetRunningPid(&pid1, &pid2);
    if (pid1 == -1 || pid2 == -1) {
      for (int pid : {pid1, pid2}) {
        if (pid == -1)
          continue;
        char pstr[16] = {0};
        sprintf(pstr, "%d", pid);
        char cmd[64] = "kill -9 ";
        strcat(cmd, pstr);

        std::cout << cmd << std::endl;
        system(cmd);
      }
      run_gps_main = std::thread(RunGPSMain0);
      run_gps_main.detach();

      run_gps_main = std::thread(RunGPSMain1);
      run_gps_main.detach();
      std::this_thread::sleep_for(2s);
    } // If should restart processed

    // Turn on light for status
    bool is_gps_main_running = (pid1 != -1 && pid2 != -1);
    std::cout << "Progress is running: " << is_gps_main_running << std::endl;
    if (is_gps_main_running) {
      for (int i = 0; i < 2; i++) {
        digitalWrite(GREEN, HIGH);
        std::this_thread::sleep_for(50ms);
        digitalWrite(GREEN, LOW);
        std::this_thread::sleep_for(50ms);
      }
      digitalWrite(GREEN, LOW);
    } else {
      for (int i = 0; i < 2; i++) {
        digitalWrite(RED, HIGH);
        std::this_thread::sleep_for(50ms);
        digitalWrite(RED, LOW);
        std::this_thread::sleep_for(50ms);
      }
      digitalWrite(RED, LOW);
    }

    std::this_thread::sleep_for(2s);
  }

  return 0;
}
