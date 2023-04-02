
protocol = ...
message = ...
MessageHeader = ...
Crc = ...

NOCRC = 0

MSG_PING = 0


@protocol
class Protocol:
    header: MessageHeader[0xAA]
    code: Code
    message = Message
    crc: Crc = NOCRC
