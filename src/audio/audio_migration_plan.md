# Migration Plan: Reliable I2S Audio Output with `pico-audio` on RP2040

## Overview

This document provides a detailed, step-by-step migration plan to transition from a callback-based I2S audio output system (as currently implemented in `i2s.h` and `mypicoOSC1.ino`) to a more robust, DMA-buffered, and IRQ-driven audio output approach using the `pico-audio` library. The migration preserves your existing pin assignments (BCLK=16, DATA=15, MCLK=17) and adapts your synthesis code to operate with buffer-based audio output, while maintaining a callback-like structure for audio generation.

---

## Why Migrate?

- **Current Issues:** The existing audio output functions only briefly, likely due to buffer underflow/overflow, DMA/PIO synchronization issues, or interrupt handling problems.
- **Advantages of `pico-audio`:**
  - Robust buffer pool management using a producer/consumer model
  - DMA-driven, IRQ-refilled audio output for improved reliability
  - Simplified debugging and enhanced error handling capabilities
  - Scalability for future features such as multi-channel support and format conversion

---

## Step-by-Step Migration Plan

### 1. **Pin and Hardware Configuration**

- Define the following macros in your build system or before including `audio_i2s.h`:
  ```c
  #define PICO_AUDIO_I2S_DATA_PIN 15
  #define PICO_AUDIO_I2S_CLOCK_PIN_BASE 16
  ```
- **Note:** The MCLK pin (17) is not managed by default in `pico-audio`. If your audio codec requires MCLK, you will need to generate it separately, for example using a PIO or PWM peripheral.

---

:start_line:33
-------
### 2. **Adapting Synthesis to Buffer-Based Model**

- Refactor your audio generation code to fill an entire audio buffer instead of outputting one sample at a time.
- Implement a function similar to the following:
  ```cpp
  void fill_audio_buffer(audio_buffer_t *buffer) {
      // Generate N stereo samples and fill buffer->buffer->bytes
      // Integrate your oscillator/phasor synthesis code here
  }
  ```
- This function will be invoked whenever a new buffer is required, preserving the callback-like behavior of your existing code.

---

:start_line:47
-------
### 3. **Producer Pool Setup**

- Define an `audio_buffer_format_t` structure for 16-bit stereo PCM at your target sample rate.
- Create a producer pool as shown below:
  ```cpp
  audio_buffer_format_t my_format = {
      .format = &(audio_format_t){
          .sample_freq = 44100,
          .format = AUDIO_BUFFER_FORMAT_PCM_S16,
          .channel_count = 2
      },
      .sample_stride = 4 // 2 channels * 2 bytes per sample
  };
  audio_buffer_pool_t *producer_pool = audio_new_producer_pool(&my_format, 3, 256);
  ```
- In your main loop or a dedicated audio task, use the following pattern:
  ```cpp
  while (1) {
      audio_buffer_t *buf = take_audio_buffer(producer_pool, true);
      fill_audio_buffer(buf);
      buf->sample_count = N; // Set the number of stereo frames filled
      give_audio_buffer(producer_pool, buf);
  }
  ```

---

:start_line:74
-------
### 4. **I2S Output Initialization**

- Initialize the I2S output with the following configuration:
  ```cpp
  audio_i2s_config_t i2s_config = {
      .data_pin = 15,
      .clock_pin_base = 16,
      .dma_channel = 0, // Select an available DMA channel
      .pio_sm = 0       // Select an available PIO state machine
  };
  audio_i2s_setup(my_format.format, &i2s_config);
  audio_i2s_connect(producer_pool);
  audio_i2s_set_enabled(true);
  ```

---

:start_line:91
-------
### 5. **Testing and Tuning**

- Adjust buffer size and count to achieve the lowest possible latency without causing underflows.
- Utilize debug hooks and assertions to detect underruns and overruns.
- Profile your audio generation code to ensure it meets real-time performance requirements.

---

## Mermaid Diagram: Data Flow

```mermaid
flowchart TD
    A[Oscillator/Phasor Synthesis] --> B[fill_audio_buffer()]
    B --> C[Producer Buffer]
    C --> D[give_audio_buffer()]
    D --> E[pico-audio DMA/IRQ]
    E --> F[PIO I2S Output]
```

---

:start_line:112
-------
## Potential Challenges & Notes

- **Callback Adaptation:** Your synthesis code must be adapted to fill audio buffers, though the core logic can remain largely unchanged.
- **Thread Safety:** When using interrupts or multicore processing, ensure that buffer access is properly synchronized to avoid race conditions.
- **DMA/PIO Conflicts:** Verify that the selected DMA channel and PIO state machine are not in use by other parts of your system to prevent conflicts.

---

:start_line:120
-------
## References

- [`pico-audio/audio_i2s.h`](pico-audio/audio_i2s.h:1)
- [`pico-audio/audio.h`](pico-audio/audio.h:1)
- [`pico-audio/audio.cpp`](pico-audio/audio.cpp:1)
- [`mypicoOSC1.ino`](mypicoOSC1.ino:1)
- [`i2s.h`](i2s.h:1)

---

**Upon successful implementation of this migration plan, you will achieve reliable, low-latency I2S audio output on the RP2040, with a robust architecture that supports future enhancements and scalability.**
