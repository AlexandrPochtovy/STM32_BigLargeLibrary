Common buffers for libraries: simple ring buffer, simple stack.

CommonDataTypes.h - data type for these buffers.

FIFO buffer: a simple ring buffer (first input - first output) library. 
Suitable for independent use, and is also used in data exchange 
libraries for i2c spi usart interfaces, etc.

FILO buffer: a simple stack (first input - last output) buffer library.
Suitable for use as data stack. 
