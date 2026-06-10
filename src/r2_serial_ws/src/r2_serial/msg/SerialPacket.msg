# Generic packet request/notification for the shared MCU serial link.
# code is the protocol message code, for example 0x0101 or 0x0311.
uint16 code

# Payload bytes are already in protocol payload order.
# For int16 fields, use little-endian bytes to match the STM32 payload convention.
uint8[] payload

# When true, the downlink node clears queued-but-not-yet-written packets before
# sending this packet. Use for emergency stop or other priority commands.
bool clear_pending
