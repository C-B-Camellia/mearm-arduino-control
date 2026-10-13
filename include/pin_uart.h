#pragma once
void initializePinUart();
int readPinUart();
// Consuming an RX error also discards queued bytes before resynchronization.
bool pinUartOverflow();
