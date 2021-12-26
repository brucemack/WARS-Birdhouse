#ifndef _CircularBuffer_h
#define _CircularBuffer_h

/** 
 *  A circular queue for byte buffers of arbitrary length. 
 */
template<unsigned int S> class CircularBuffer {
public:

  CircularBuffer() 
  : _front(0),
    _back(0)
  {
  }

  bool isEmpty() {
    return _front == _back;
  }

  // Returns true if the push was successful, otherwise false.  For example, if the buffer
  // is completely full.
  // NOTE: A two-byte length header is put into the buffer to manage size.
  bool push(const uint8_t* buf, unsigned int len) {
    // Keep the original pointer in case a failure/rollback is necessary.
    unsigned int original_back = _back;
    // Write a two-byte length
    uint8_t temp[2];
    temp[0] = (len >> 8) & 0xff;
    temp[1] = len & 0xff;
    if (_pushRaw(temp, 2) && _pushRaw(buf, len)) {
      return true;
    }
    // Reset like nothing happened
    _back = original_back;
    return false;
  }

  // The *len argument starts off with the maximum space available in buf and ends
  // with the actual number of bytes taken from the queue.
  void pop(uint8_t* buf, unsigned int* len) {
    // Get the next buffer and move pointer 
    _front = _peek(buf, len);
  }

  void peek(uint8_t* buf, unsigned int* len) {
    // Get the next buffer but don't move the pointer
    _peek(buf, len);
  }

  /**
   * Removes and discards the front message from the circular queue.
   */
  void popAndDiscard() {
    unsigned int ptr = _front;
    // Get out the length
    unsigned int entry_size = 0;
    entry_size = _buf[ptr] << 8;
    ptr = (ptr + 1) % _bufSize;
    entry_size |= _buf[ptr];
    ptr = (ptr + 1) % _bufSize;  
    // Rotate through the message
    for (int i = 0; i < entry_size; i++) {
      ptr = (ptr + 1) % _bufSize;
    }
    // Reset the front pointer
    _front = ptr;
  }
    
  /**
   * This combines isEmpty() and pop() to provide an easy atomic 
   * pop operation.
   */
  bool popIfNotEmpty(uint8_t* buf, unsigned int* len) {
    if (isEmpty()) {
      return false;
    } else {
      pop(buf, len);
      return true;
    }
  }
  
private:

  const unsigned int _bufSize = S;

  // Where we pop (read) from
  volatile unsigned int _front;
  // Where we push (write) to
  volatile unsigned int _back;
  // The actual space
  volatile uint8_t _buf[S];

  bool _pushRaw(const uint8_t* buf, unsigned int len) {
    for (unsigned int i = 0; i < len; i++) {
      // Store into the buffer
      _buf[_back] = buf[i];
      // Advance and wrap
      _back = (_back + 1) % _bufSize;
      // Check for overflow
      if (_back == _front) {
        return false;
      }
    }
    return true;
  }

  // The *len argument starts off with the maximum space available in buf and ends
  // with the actual number of bytes taken from the queue.
  // Returns the location of the new front of the queue.
  unsigned int _peek(uint8_t* buf, unsigned int* len) {
    unsigned int available_space = *len;
    unsigned int ptr = _front;
    // Get out the length
    unsigned int entry_size = 0;
    entry_size = _buf[ptr] << 8;
    ptr = (ptr + 1) % _bufSize;
    entry_size |= _buf[ptr];
    ptr = (ptr + 1) % _bufSize;
    
    *len = 0;
    
    for (int i = 0; i < entry_size; i++) {
      if (i < available_space) {
        buf[i] = _buf[ptr];
        (*len)++;
      }
      ptr = (ptr + 1) % _bufSize;
    }

    return ptr;
  }
};

#endif
