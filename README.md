# FRDM-MCXN947 FreeRTOS RC Receiver (FS-iA6B)

This project implements a Radio Control (RC) receiver parser for the NXP **FRDM-MCXN947** development board using **FreeRTOS**. It is specifically designed to interface with the **FlySky FS-iA6B** receiver over a UART interface using EDMA (Enhanced Direct Memory Access) for highly efficient, non-blocking data reception.

## Features

* **FreeRTOS Integration:** Uses a dedicated FreeRTOS task (`RCParserTask`) to handle incoming receiver data.
* **Non-blocking UART + EDMA:** Leverages the NXP MCUXpresso SDK's EDMA driver to receive 32-byte IBUS frames without CPU intervention, minimizing overhead.
* **FlySky FS-iA6B Support:** Parses the 14-channel IBUS protocol used by FlySky receivers.
* **Hardware Abstraction:** Modular UART driver (`uart_driver_mcxn947.c`) that can be easily configured for different LPUART instances and DMA channels.

## Hardware Requirements

* **Board:** NXP FRDM-MCXN947
* **Receiver:** FlySky FS-iA6B (or compatible IBUS receiver)
* **Connections:** 
  * Connect the receiver's IBUS/IBUS output to the configured LPUART RX pin on the MCXN947.
  * *Note: By default, this project is configured to use `LPUART1` and `DMA0` (Channel 1).*

## Project Structure

* `freertos_hello.c`: Main application entry point. Initializes hardware, creates the FreeRTOS tasks, and implements the `RCParserTask` that waits for DMA transfer completion notifications.
* `rc_fsia6B.c` / `.h`: The FS-iA6B protocol driver. Handles the initialization of the UART/DMA specifically for the RC parsing and triggers DMA ring buffer operations.
* `uart_driver_mcxn947.c` / `.h`: A customized, interrupt/DMA-driven UART abstraction layer for the MCXN947.

## Building and Running

This project uses CMake. You can build it directly from VS Code using the provided CMake Tools integrations:

1. Open the workspace in VS Code.
2. Ensure you have the required ARM GCC toolchain and NXP SDK configured.
3. Run the **CMake: build** task (or build preset).
4. Flash the generated executable to the FRDM-MCXN947 board using your preferred debug probe (e.g., J-Link or CMSIS-DAP).

## Architecture Details

1. **Initialization:** `rc_init()` configures the target LPUART and binds the EDMA callback.
2. **Reception:** `rc_start_dma_rx()` kicks off an asynchronous DMA read for exactly `FS_IA6B_FRAME_SIZE` (32) bytes.
3. **Synchronization:** When 32 bytes are received, the `RC_Callback` (running in the ISR context) unblocks the `RCParserTask` using `vTaskNotifyGiveFromISR()`.
4. **Processing:** The task wakes up, accesses the `g_rxBuffer`, prints/parses the frame, and safely restarts the DMA reception queue.
