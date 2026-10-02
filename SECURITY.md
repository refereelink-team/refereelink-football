# Security

The current development line is `main`; older snapshots are not a supported release series.

Report vulnerabilities privately using GitHub **Security → Report a vulnerability** on this repository. Provide the affected commit, a minimal reproduction and impact, excluding credentials, device IDs and private captures. Do not put exploit details or personal data in public issues. No response-time guarantee is made.

## Integration boundary

UART input is untrusted. Keep bounded buffers, validate complete frames and preserve overrun/error counters. The UWB vendor frame has no checksum; framing checks cannot prove radio data authenticity or detect every corruption. The software performs no transport encryption or authentication. Apply those controls in the board/host transport where required.

This repository is a portable software layer, not a validated physical board or ready-to-flash firmware. Confirm voltage levels, pins, clocks, interrupt behavior and watchdog/error handling on the actual hardware. Keep manufacturer firmware/SDK licensing separate from this original MIT-licensed code.
