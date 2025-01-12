#include "esp_system.h"
#include "esp_debug_helpers.h"
#include "esp32-hal-log.h"
#include "esp_attr.h"
#include "esp_err.h"

extern "C" void IRAM_ATTR __wrap_esp_panic_handler(void* info) {
  log_printf("Custom panic handler called!\n");

  if (info) {
    const void** pinfo = (const void**)info;
    log_printf("Crash address: 0x%08x\n", (uint32_t)pinfo[0]);
    log_printf("Crash reason: %d\n", (int)pinfo[1]);
  }

  // Print the exception registers
  uint32_t *regs = (uint32_t *)((uint8_t *)info + 3 * sizeof(void*));
  log_printf("Exception registers:\n");
  for (int i = 0; i < 24; i++) {
    log_printf("Reg[%d] = 0x%08x\n", i, regs[i]);
  }

  // Print the backtrace
  log_printf("Backtrace:\n");
  esp_backtrace_print(100);

  // Print free heap
  log_printf("Free heap: %u\n", esp_get_free_heap_size());

  // Print minimum free heap
  log_printf("Minimum free heap: %u\n", esp_get_minimum_free_heap_size());

  // Call the original panic handler
  //extern void __real_esp_panic_handler(void* info);
  //__real_esp_panic_handler(info);
  while (true) ;
}
