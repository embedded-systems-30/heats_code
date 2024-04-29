
#ifndef __BLUETOOTH_HPP
#define __BLUETOOTH_HPP

#include "mbed.h"
#include <vector>

/**
 * @brief Event-based HM-10 (Bluetooth/UART emulation) module.
 * 
 * See following code example for usage:
 * @include bluetoothExample.cpp
 */
class Bluetooth {
public:
  /**
   * @brief Single byte bluetooth command
   *
   */
  struct Command {
    /** Command opcode (operands should be zero'd out in cmd AND mask.) */
    uint8_t cmd;
    /** Opcode mask (all bits not covered by the mask will not be matched
     * against cmd.)*/
    uint8_t mask;
    /**
     * Callback to execute on matching command input
     *
     * @note This is executed in mbed shared queue context, so while it is okay
     * to use blocking calls, you should avoid using lots of time to allow other
     * events through.
     */
    Callback<void(uint8_t)> action;
    Command(uint8_t cmd, uint8_t mask, Callback<void(uint8_t)> action);
  };
  /**
   * @brief Construct a new Bluetooth object
   *
   * @param serial Serial interface corresponding to Bluetooth.
   * @param userQueue Queue that runs in user context for processing to
   * occur in.
   * @param commands Initial command set to use
   * @param startNow Set to true to start waiting for incoming commands.
   */
  explicit Bluetooth(BufferedSerial *serial, EventQueue *,
                     std::vector<Command> commands = {}, bool startNow = false);
  /**
   * @brief Register a new command to the bluetooth object.
   * @note Bluetooth command should be fully initialized.
   * @param cmd Bluetooth::Command to register
   */
  void addCommand(Bluetooth::Command cmd);
  /**
   * @brief Deregister a bluetooth command
   * @note Avoid excessive usage, commands have to be searched for.
   * @param cmd Command to deregister
   */
  void removeCommand(Bluetooth::Command *cmd);
  void removeCommand(uint8_t cmd);
  void start(); /**< Start waiting for bluetooth commands */
  void stop(); /**< Stop waiting for bluetooth commands */

private:
  EventQueue *deferQueue; /**< Queue to execute blocking actions in. */
  BufferedSerial *s; /**< BLE UART Serial Interface*/
  std::vector<Command> commands; /**< Vector of all registered commands*/
  /**
   * @brief Attaches to the BufferedSerial's `sigio` event.
   * @note Executes in ISR context!
   */
  void onSigio();
  /**
   * @brief Reads the incoming bluetooth commands and flushes
   * the queue.
   * @note This is called by `onSigio` by deferred call into the shared event queue.
   */
  void poll();
  /**
   * @brief Parse a byte into a corresponding registered bluetooth command.
   * 
   * @param instruction Byte to parse
   */
  void commandParser(uint8_t instruction);
};

#endif // __BLUETOOTH_HPP
