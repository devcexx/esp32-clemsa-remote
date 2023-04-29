#ifndef CLEMSACODE_H
#define CLEMSACODE_H
#include "driver/gptimer.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>
#include "driver/gpio.h"

#define CLEMSA_CODEGEN_DEFAULT_CODE_SIZE 36

/* Number of base clock cycles to generate before starting sending the
   code itself. */
#define CLEMSA_CODEGEN_SYNC_CLOCK_CYCLES 79

/* Number of base clock cycles between the end of the transmission of
   the sync signal and the beginning of the code transmission */
#define CLEMSA_CODEGEN_WAIT_CLOCK_CYCLES 5

/* Number of base clock cycles between the end of the transmission of
   the code and the beginning of a repetition */
#define CLEMSA_CODEGEN_CYCLES_BETWEEN_REPETITIONS 5

/* The resolution of the base clock. Base clock will take 1 / (this
   value) seconds to count one. */
#define CLEMSA_CODEGEN_BASE_CLK_RESOLUTION 1000000

/* The frequency of the ASK generator */
#define CLEMSA_CODEGEN_ASK_CLK_FREQUENCY 16000

/* The time the base clock will generate a high signal, and low
   signal, respectively, expressed as the number of cycles of the base
   clock at a frequency of CLEMSA_CODEGEN_BASE_CLK_RESOLUTION. This
   will use to generate a PWM pulse in wich the cycle width will be
   CLEMSA_CODEGEN_CLK_HIGH_COUNT + CLEMSA_CODEGEN_CLK_LOW_COUNT, and
   the duty, CLEMSA_CODEGEN_CLK_HIGH_COUNT /
   (CLEMSA_CODEGEN_CLK_HIGH_COUNT + CLEMSA_CODEGEN_CLK_LOW_COUNT) */
#define CLEMSA_CODEGEN_CLK_HIGH_COUNT 2080
#define CLEMSA_CODEGEN_CLK_LOW_COUNT 1040

/* The number of the ASK ticks that needs to be emitted for each digit
   of the transmitting code */
#define CLEMSA_CODEGEN_ASK_TICKS_ZERO 15
#define CLEMSA_CODEGEN_ASK_TICKS_ONE 100

#if (CLEMSA_CODEGEN_CLK_LOW_COUNT >= CLEMSA_CODEGEN_CLK_HIGH_COUNT)
#error "CLEMSA_CODEGEN_CLK_LOW_COUNT cannot be greater or equal than CLEMSA_CODEGEN_CLK_HIGH_COUNT"
#endif

#define CLEMSA_CODEGEN

struct clemsa_codegen {
  gpio_num_t gpio;

  /* (Internal) Base clock that drives the code generation */
  gptimer_handle_t _base_clk;

  /* (Internal) 16 kHz clock that generates the ASK pulse */
  gptimer_handle_t _ask_clk;
};

struct clemsa_code_tx {
  /* An identificative name for the code */
  const char* code_name;

  /* The code that will be sent */
  const bool* code;

  /* The length of the code that will be sent */
  size_t code_len;

  /* Number of times that the code will be repeated */
  uint32_t repetition_count;

  /* The code generator */
  struct clemsa_codegen* _generator;

  /* (Internal) the next digit of the code that needs to be sent */
  volatile size_t _next_digit;

  /* (Internal) the current elapsed base clock cycles from when the
     transmission started */
  volatile uint32_t _base_clk_cycles;

  /* (Internal) indicates whether we're currently on the high section
     of the pulse of the base clock */
  volatile bool _base_clk_high;

  /* (Internal) indicates whether the ASK output is currently high */
  volatile bool _ask_clk_high;

  /* (Internal) When the ASK clock is enabled, holds how many more ticks should
     oscillate the transmission until it autostops */
  volatile int _remaining_ask_ticks;

  /* (Internal) Indicates when will the next code repetition start to be transmitted. */
  uint32_t _next_code_repetition_start_cycle;

  /* (Internal) Number of repetitions sent */
  uint32_t _times_code_sent;

  volatile bool _terminated;
};

esp_err_t clemsa_codegen_init(struct clemsa_codegen *ptr, gpio_num_t gpio);
//esp_err_t clemsa_codegen_deinit(struct clemsa_codegen *ptr, gpio_num_t gpio);
esp_err_t clemsa_codegen_begin_tx(struct clemsa_codegen *instance, struct clemsa_code_tx* tx);
#endif /* CLEMSACODE_H */
