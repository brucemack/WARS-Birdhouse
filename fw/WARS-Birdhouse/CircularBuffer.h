/* 
 * LoRa Birdhouse Mesh Network Project
 * Wellesley Amateur Radio Society
 * 
 * Copyright (C) 2022 Bruce MacKinnon
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef _CircularBuffer_h
#define _CircularBuffer_h

#include <stdint.h>

/**
 * @brief Base interface for the circular buffer concept.
 * 
 */
class CircularBuffer {
public:

  virtual bool isEmpty() const  = 0;
  virtual bool push(const void* oobBuf, const void* buf, unsigned int bufLen) = 0;
  virtual void pop(void* oobBuf, void* buf, unsigned int* len) = 0;
  virtual bool popIfNotEmpty(void* oobBuf, void* buf, unsigned int* len) = 0;
  virtual void peek(void* oobBuf, void* buf, unsigned int* len) const = 0;
  virtual void popAndDiscard() = 0;
};

/** 
 *  A circular queue for byte buffers of arbitrary length. 
 */
template<unsigned int S> class CircularBufferImpl : CircularBuffer {
public:

  CircularBufferImpl(unsigned int oobBufLen) 
  : _front(0),
    _back(0),
    _oobBufLen(oobBufLen)
  {
  }

  bool isEmpty() const {
    return _front == _back;
  }

  /**
   * @brief Push a fixed-length OOB buffer and a variable length
   * buffer onto the circular buffer.
   * 
   * Returns true if the push was successful, otherwise false.  For example, 
   * if the buffer is completely full a push will fail.
   *
   * NOTE: A two-byte length header is put into the buffer to manage size.
   */
  bool push(const void* oobBuf, const void* buf, unsigned int bufLen) {
    // Keep the original pointer in case a failure/rollback is necessary.
    unsigned int originalBack = _back;
    // We are putting the combined length of the OOB and IB buffers
    // onto the circular buffer
    unsigned int totalLen = bufLen + _oobBufLen;
    // Write a two-byte length
    uint8_t temp[2];
    temp[0] = (totalLen >> 8) & 0xff;
    temp[1] = totalLen & 0xff;
    // Push the different components onto the circular buffer
    if (_pushRaw(temp, 2) && 
        _pushRaw(oobBuf, _oobBufLen) && 
        _pushRaw(buf, bufLen)) {
      return true;
    }
    // If anything fails then reset like nothing happened
    _back = originalBack;
    // Return the error condition
    return false;
  }

  // The *len argument starts off with the maximum space available in buf and ends
  // with the actual number of bytes taken from the queue.
  void pop(void* oobBuf, void* buf, unsigned int* len) {
    // Get the next buffer and move pointer 
    _front = _peek(oobBuf, buf, len);
  }

  void peek(void* oobBuf, void* buf, unsigned int* len) const {
    // Get the next buffer but don't move the pointer
    _peek(oobBuf, buf, len);
  }

  static unsigned int _incAndWrap(unsigned int ptr) {
    return (ptr + 1) % _bufLen;
  }

  /**
   * Removes and discards the front message from the circular queue.
   */
  void popAndDiscard() {
    unsigned int ptr = _front;
    
    // Get out the length (first two bytes)
    unsigned int entry_size = 0;
    entry_size = _buf[ptr] << 8;
    ptr = _incAndWrap(ptr);
    entry_size |= _buf[ptr];
    ptr = _incAndWrap(ptr);

    // Rotate through the OOB section and the IB section
    for (unsigned int i = 0; i < entry_size; i++) {
      ptr = _incAndWrap(ptr);
    }

    // Reset the front pointer
    _front = ptr;
  }
    
  /**
   * This combines isEmpty() and pop() to provide an easy atomic 
   * pop operation.
   */
  bool popIfNotEmpty(void* oobBuf, void* buf, unsigned int* len) {
    if (isEmpty()) {
      return false;
    } else {
      pop(oobBuf, buf, len);
      return true;
    }
  }
  
private:

  static const unsigned int _bufLen = S;

  // The size of the OOB header
  const unsigned int _oobBufLen;
  // Where we pop (read) from
  volatile unsigned int _front;
  // Where we push (write) to
  volatile unsigned int _back;
  // The actual space
  volatile uint8_t _buf[S];

  /**
   * @brief This puts an entry onto the circular buffer, making 
   * sure that a wrap-around situation doesn't happen.
   * 
   * @param oobBuf 
   * @param buf 
   * @param len 
   * @return true 
   * @return false 
   */
  bool _pushRaw(const void* buf, unsigned int bufLen) {
    // Deal with the variable length IB part    
    for (unsigned int i = 0; i < bufLen; i++) {
      // Store into the buffer
      _buf[_back] = ((uint8_t*)buf)[i];
      // Advance and wrap
      _back = _incAndWrap(_back);
      // Check for overflow
      if (_back == _front) {
        return false;
      }
    }
    // If we get here then the push was successful
    return true;
  }

  // The *bufLen argument starts off with the maximum space available in buf and ends
  // with the actual number of bytes taken from the queue.
  // Returns the location of the new front of the queue.
  unsigned int _peek(void* oobBuf, void* buf, unsigned int* bufLen) const {

    unsigned int ptr = _front;

    // Get out the length (inclusive of OOB and IB parts)
    unsigned int entry_size = 0;
    entry_size = _buf[ptr] << 8;
    ptr = _incAndWrap(ptr);
    entry_size |= _buf[ptr];
    ptr = _incAndWrap(ptr);

    // Get the OOB data
    for (unsigned int i = 0; i < _oobBufLen; i++) {
      ((uint8_t*)oobBuf)[i] = _buf[ptr];
      ptr = _incAndWrap(ptr);
      entry_size--;
    }

    // Capture the maximum space available
    unsigned int available_space = *bufLen;
    *bufLen = 0;
    
    for (int i = 0; i < entry_size; i++) {
      // Notice that when we exceed the maximum space we 
      // just start quietly ignoring the IB data.
      if (i < available_space) {
        ((uint8_t*)buf)[i] = _buf[ptr];
        (*bufLen)++;
      }
      ptr = _incAndWrap(ptr);
    }

    return ptr;
  }
};

#endif
