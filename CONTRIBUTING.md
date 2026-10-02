# Contributing

Open a focused pull request describing the protocol or integration problem, implementation and test evidence. Keep manufacturer links and original-code attribution. Do not copy vendor firmware, SDKs or captures without redistribution rights; do not submit device IDs, credentials or private deployment configuration.

Run checks on a remote Linux host with a C11 compiler, make and Python 3.12+:

```sh
make test
make clean
make build/test_telemetry build/test_hal_adapter CFLAGS='-std=c11 -g -O1 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer'
./build/test_telemetry
./build/test_hal_adapter
```

CI runs the same software checks. Synthetic frames and HAL stubs do not establish real UART timing, board compatibility or ranging accuracy. For a hardware contribution, record MCU/board model, voltage levels, physical pin mapping, UART configuration and measured behavior. Keep transport channels separate and bound ISR/process work. No ready-to-flash board project is implied by the portable examples.
