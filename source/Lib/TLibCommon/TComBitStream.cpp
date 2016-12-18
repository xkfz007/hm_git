<<<<<<< HEAD
/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComBitStream.cpp
    \brief    class for handling bitstream
*/

#include <stdint.h>
#include <vector>
#include "TComBitStream.h"
#include <string.h>
#include <memory.h>

using namespace std;

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComOutputBitstream::TComOutputBitstream()
{
  clear();
}

TComOutputBitstream::~TComOutputBitstream()
{
}


TComInputBitstream::TComInputBitstream()
: m_fifo()
, m_emulationPreventionByteLocation()
, m_fifo_idx(0)
, m_num_held_bits(0)
, m_held_bits(0)
, m_numBitsRead(0)
{ }

TComInputBitstream::TComInputBitstream(const TComInputBitstream &src)
: m_fifo(src.m_fifo)
, m_emulationPreventionByteLocation(src.m_emulationPreventionByteLocation)
, m_fifo_idx(src.m_fifo_idx)
, m_num_held_bits(src.m_num_held_bits)
, m_held_bits(src.m_held_bits)
, m_numBitsRead(src.m_numBitsRead)
{ }

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TComInputBitstream::resetToStart()
{
  m_fifo_idx=0;
  m_num_held_bits=0;
  m_held_bits=0;
  m_numBitsRead=0;
}

Char* TComOutputBitstream::getByteStream() const
{
  return (Char*) &m_fifo.front();
}

UInt TComOutputBitstream::getByteStreamLength()
{
  return UInt(m_fifo.size());
}

Void TComOutputBitstream::clear()
{
  m_fifo.clear();
  m_held_bits = 0;
  m_num_held_bits = 0;
}

Void TComOutputBitstream::write   ( UInt uiBits, UInt uiNumberOfBits )
{
  assert( uiNumberOfBits <= 32 );
  assert( uiNumberOfBits == 32 || (uiBits & (~0 << uiNumberOfBits)) == 0 );

  /* any modulo 8 remainder of num_total_bits cannot be written this time,
   * and will be held until next time. */
  UInt num_total_bits = uiNumberOfBits + m_num_held_bits;
  UInt next_num_held_bits = num_total_bits % 8;

  /* form a byte aligned word (write_bits), by concatenating any held bits
   * with the new bits, discarding the bits that will form the next_held_bits.
   * eg: H = held bits, V = n new bits        /---- next_held_bits
   * len(H)=7, len(V)=1: ... ---- HHHH HHHV . 0000 0000, next_num_held_bits=0
   * len(H)=7, len(V)=2: ... ---- HHHH HHHV . V000 0000, next_num_held_bits=1
   * if total_bits < 8, the value of v_ is not used */
  UChar next_held_bits = uiBits << (8 - next_num_held_bits);

  if (!(num_total_bits >> 3))
  {
    /* insufficient bits accumulated to write out, append new_held_bits to
     * current held_bits */
    /* NB, this requires that v only contains 0 in bit positions {31..n} */
    m_held_bits |= next_held_bits;
    m_num_held_bits = next_num_held_bits;
    return;
  }

  /* topword serves to justify held_bits to align with the msb of uiBits */
  UInt topword = (uiNumberOfBits - next_num_held_bits) & ~((1 << 3) -1);
  UInt write_bits = (m_held_bits << topword) | (uiBits >> next_num_held_bits);

  switch (num_total_bits >> 3)
  {
  case 4: m_fifo.push_back(write_bits >> 24);
  case 3: m_fifo.push_back(write_bits >> 16);
  case 2: m_fifo.push_back(write_bits >> 8);
  case 1: m_fifo.push_back(write_bits);
  }

  m_held_bits = next_held_bits;
  m_num_held_bits = next_num_held_bits;
}

Void TComOutputBitstream::writeAlignOne()
{
  UInt num_bits = getNumBitsUntilByteAligned();
  write((1 << num_bits) - 1, num_bits);
  return;
}

Void TComOutputBitstream::writeAlignZero()
{
  if (0 == m_num_held_bits)
  {
    return;
  }
  m_fifo.push_back(m_held_bits);
  m_held_bits = 0;
  m_num_held_bits = 0;
}

/**
 - add substream to the end of the current bitstream
 .
 \param  pcSubstream  substream to be added
 */
Void   TComOutputBitstream::addSubstream( TComOutputBitstream* pcSubstream )
{
  UInt uiNumBits = pcSubstream->getNumberOfWrittenBits();

  const vector<uint8_t>& rbsp = pcSubstream->getFIFO();
  for (vector<uint8_t>::const_iterator it = rbsp.begin(); it != rbsp.end();)
  {
    write(*it++, 8);
  }
  if (uiNumBits&0x7)
  {
    write(pcSubstream->getHeldBits()>>(8-(uiNumBits&0x7)), uiNumBits&0x7);
  }
}

Void TComOutputBitstream::writeByteAlignment()
{
  write( 1, 1);
  writeAlignZero();
}

Int TComOutputBitstream::countStartCodeEmulations()
{
  UInt cnt = 0;
  vector<uint8_t>& rbsp   = getFIFO();
  for (vector<uint8_t>::iterator it = rbsp.begin(); it != rbsp.end();)
  {
    vector<uint8_t>::iterator found = it;
    do
    {
      // find the next emulated 00 00 {00,01,02,03}
      // NB, end()-1, prevents finding a trailing two byte sequence
      found = search_n(found, rbsp.end()-1, 2, 0);
      found++;
      // if not found, found == end, otherwise found = second zero byte
      if (found == rbsp.end())
      {
        break;
      }
      if (*(++found) <= 3)
      {
        break;
      }
    } while (true);
    it = found;
    if (found != rbsp.end())
    {
      cnt++;
    }
  }
  return cnt;
}

/**
 * read uiNumberOfBits from bitstream without updating the bitstream
 * state, storing the result in ruiBits.
 *
 * If reading uiNumberOfBits would overrun the bitstream buffer,
 * the bitstream is effectively padded with sufficient zero-bits to
 * avoid the overrun.
 */
Void TComInputBitstream::pseudoRead ( UInt uiNumberOfBits, UInt& ruiBits )
{
  UInt saved_num_held_bits = m_num_held_bits;
  UChar saved_held_bits = m_held_bits;
  UInt saved_fifo_idx = m_fifo_idx;

  UInt num_bits_to_read = min(uiNumberOfBits, getNumBitsLeft());
  read(num_bits_to_read, ruiBits);
  ruiBits <<= (uiNumberOfBits - num_bits_to_read);

  m_fifo_idx = saved_fifo_idx;
  m_held_bits = saved_held_bits;
  m_num_held_bits = saved_num_held_bits;
}


Void TComInputBitstream::read (UInt uiNumberOfBits, UInt& ruiBits)
{
  assert( uiNumberOfBits <= 32 );

  m_numBitsRead += uiNumberOfBits;

  /* NB, bits are extracted from the MSB of each byte. */
  UInt retval = 0;
  if (uiNumberOfBits <= m_num_held_bits)
  {
    /* n=1, len(H)=7:   -VHH HHHH, shift_down=6, mask=0xfe
     * n=3, len(H)=7:   -VVV HHHH, shift_down=4, mask=0xf8
     */
    retval = m_held_bits >> (m_num_held_bits - uiNumberOfBits);
    retval &= ~(0xff << uiNumberOfBits);
    m_num_held_bits -= uiNumberOfBits;
    ruiBits = retval;
    return;
  }

  /* all num_held_bits will go into retval
   *   => need to mask leftover bits from previous extractions
   *   => align retval with top of extracted word */
  /* n=5, len(H)=3: ---- -VVV, mask=0x07, shift_up=5-3=2,
   * n=9, len(H)=3: ---- -VVV, mask=0x07, shift_up=9-3=6 */
  uiNumberOfBits -= m_num_held_bits;
  retval = m_held_bits & ~(0xff << m_num_held_bits);
  retval <<= uiNumberOfBits;

  /* number of whole bytes that need to be loaded to form retval */
  /* n=32, len(H)=0, load 4bytes, shift_down=0
   * n=32, len(H)=1, load 4bytes, shift_down=1
   * n=31, len(H)=1, load 4bytes, shift_down=1+1
   * n=8,  len(H)=0, load 1byte,  shift_down=0
   * n=8,  len(H)=3, load 1byte,  shift_down=3
   * n=5,  len(H)=1, load 1byte,  shift_down=1+3
   */
  UInt aligned_word = 0;
  UInt num_bytes_to_load = (uiNumberOfBits - 1) >> 3;
  assert(m_fifo_idx + num_bytes_to_load < m_fifo.size());

  switch (num_bytes_to_load)
  {
  case 3: aligned_word  = m_fifo[m_fifo_idx++] << 24;
  case 2: aligned_word |= m_fifo[m_fifo_idx++] << 16;
  case 1: aligned_word |= m_fifo[m_fifo_idx++] <<  8;
  case 0: aligned_word |= m_fifo[m_fifo_idx++];
  }

  /* resolve remainder bits */
  UInt next_num_held_bits = (32 - uiNumberOfBits) % 8;

  /* copy required part of aligned_word into retval */
  retval |= aligned_word >> next_num_held_bits;

  /* store held bits */
  m_num_held_bits = next_num_held_bits;
  m_held_bits = aligned_word;

  ruiBits = retval;
}

/**
 * insert the contents of the bytealigned (and flushed) bitstream src
 * into this at byte position pos.
 */
Void TComOutputBitstream::insertAt(const TComOutputBitstream& src, UInt pos)
{
  UInt src_bits = src.getNumberOfWrittenBits();
  assert(0 == src_bits % 8);

  vector<uint8_t>::iterator at = m_fifo.begin() + pos;
  m_fifo.insert(at, src.m_fifo.begin(), src.m_fifo.end());
}

UInt TComInputBitstream::readOutTrailingBits ()
{
  UInt count=0;
  UInt uiBits = 0;

  while ( ( getNumBitsLeft() > 0 ) && (getNumBitsUntilByteAligned()!=0) )
  {
    count++;
    read ( 1, uiBits );
  }
  return count;
}
//
//TComOutputBitstream& TComOutputBitstream::operator= (const TComOutputBitstream& src)
//{
//  vector<uint8_t>::iterator at = m_fifo.begin();
//  m_fifo.insert(at, src.m_fifo.begin(), src.m_fifo.end());
//
//  m_num_held_bits             = src.m_num_held_bits;
//  m_held_bits                 = src.m_held_bits;
//
//  return *this;
//}

/**
 Extract substream from the current bitstream.

 \param  uiNumBits    number of bits to transfer
 */
TComInputBitstream *TComInputBitstream::extractSubstream( UInt uiNumBits )
{
  UInt uiNumBytes = uiNumBits/8;
  TComInputBitstream *pResult = new TComInputBitstream;

  std::vector<uint8_t> &buf = pResult->getFifo();
  buf.reserve((uiNumBits+7)>>3);

  if (m_num_held_bits == 0)
  {
    std::size_t currentOutputBufferSize=buf.size();
    const UInt uiNumBytesToReadFromFifo = std::min<UInt>(uiNumBytes, (UInt)m_fifo.size() - m_fifo_idx);
    buf.resize(currentOutputBufferSize+uiNumBytes);
    memcpy(&(buf[currentOutputBufferSize]), &(m_fifo[m_fifo_idx]), uiNumBytesToReadFromFifo); m_fifo_idx+=uiNumBytesToReadFromFifo;
    if (uiNumBytesToReadFromFifo != uiNumBytes)
    {
      memset(&(buf[currentOutputBufferSize+uiNumBytesToReadFromFifo]), 0, uiNumBytes - uiNumBytesToReadFromFifo);
    }
  }
  else
  {
    for (UInt ui = 0; ui < uiNumBytes; ui++)
    {
      UInt uiByte;
      read(8, uiByte);
      buf.push_back(uiByte);
    }
  }
  if (uiNumBits&0x7)
  {
    UInt uiByte = 0;
    read(uiNumBits&0x7, uiByte);
    uiByte <<= 8-(uiNumBits&0x7);
    buf.push_back(uiByte);
  }
  return pResult;
}

UInt TComInputBitstream::readByteAlignment()
{
  UInt code = 0;
  read( 1, code );
  assert(code == 1);

  UInt numBits = getNumBitsUntilByteAligned();
  if(numBits)
  {
    assert(numBits <= getNumBitsLeft());
    read( numBits, code );
    assert(code == 0);
  }
  return numBits+1;
}

//! \}
=======
/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and   contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2010, SAMSUNG ELECTRONICS CO., LTD. and BRITISH BROADCASTING CORPORATION
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within the Joint Collaborative Team on Video Coding and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * Neither the name of SAMSUNG ELECTRONICS CO., LTD. nor the name of the BRITISH BROADCASTING CORPORATION
      may be used to endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 * ====================================================================================================================
*/

/** \file     TComBitStream.cpp
    \brief    class for handling bitstream
*/

#include "TComBitStream.h"
#include <memory.h>

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

Void TComBitstream::create( UInt uiSizeInBytes )
{
  UInt uiSize = uiSizeInBytes / sizeof(UInt);
  
  m_apulStreamPacketBegin = new UInt[uiSize];
  m_uiBufSize       = uiSize;
  m_uiBitSize       = 0;
  m_iValidBits      = 32;

  m_ulCurrentBits   = 0;
  m_uiBitsWritten   = 0;

  m_pulStreamPacket = m_apulStreamPacketBegin;
}

Void TComBitstream::destroy()
{
  delete [] m_apulStreamPacketBegin;     m_apulStreamPacketBegin = NULL;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TComBitstream::write   ( UInt uiBits, UInt uiNumberOfBits )
{
  assert( m_uiBufSize > 0 );
  assert( uiNumberOfBits <= 32 );
  assert( ! ( (uiBits >> 1) >> (uiNumberOfBits - 1)) ); // because shift with 32 has no effect

  m_uiBitsWritten += uiNumberOfBits;

  if( (Int)uiNumberOfBits < m_iValidBits)  // one word
  {
    m_iValidBits -= uiNumberOfBits;

    m_ulCurrentBits |= uiBits << m_iValidBits;

    return;
  }

  UInt uiShift = uiNumberOfBits - m_iValidBits;

  // add the last bits
  m_ulCurrentBits |= uiBits >> uiShift;

  *m_pulStreamPacket++ = xSwap( m_ulCurrentBits );


  // note: there is a problem with left shift with 32
  m_iValidBits = 32 - uiShift;

  m_ulCurrentBits = uiBits << m_iValidBits;

  if( 0 == uiShift )
  {
    m_ulCurrentBits = 0;
  }
}

Void TComBitstream::writeAlignOne()
{
  write( ( 1 << (m_iValidBits & 0x7) ) - 1, m_iValidBits & 0x7 );
  return;
}

Void TComBitstream::writeAlignZero()
{
  write( 0, m_iValidBits & 0x7 );
  return;
}

Void  TComBitstream::flushBuffer()
{
  if (m_iValidBits == 0)
    return;

  *m_pulStreamPacket = xSwap( m_ulCurrentBits );

  m_uiBitsWritten = (m_uiBitsWritten+7)/8;

  m_uiBitsWritten *= 8;
}

Void TComBitstream::initParsing ( UInt uiNumBytes )
{
  m_ulCurrentBits     = 0xdeaddead;
  m_uiNextBits        = 0xdeaddead;
  m_uiBitsLeft        = 0;
  m_iValidBits        = 0;
  m_uiDWordsLeft      = 0;

  m_uiBitsLeft        = uiNumBytes << 3;

  m_uiDWordsLeft      = m_uiBitsLeft >> 5;
  m_iValidBits        = -32;

  xReadNextWord();
  xReadNextWord();
}

Void TComBitstream::read (UInt uiNumberOfBits, UInt& ruiBits)
{
  UInt ui_right_shift;

  // check the number_of_bits parameter matches the range
  assert( uiNumberOfBits <= 32 );

  if( uiNumberOfBits > m_uiBitsLeft )
  {
    assert (0);
  }

  m_uiBitsLeft -= uiNumberOfBits;
  m_iValidBits -= uiNumberOfBits;

  if( 0 <= m_iValidBits )
  {
    // calculate the number of bits to extract the desired number of bits
    ui_right_shift = 32 - uiNumberOfBits ;

    // mask out the value
    ruiBits  = m_ulCurrentBits >> ui_right_shift;

    //prepare for next access
    m_ulCurrentBits = m_ulCurrentBits << uiNumberOfBits;
  }
  else
  {
    // mask out the value in the current word
    ruiBits = m_ulCurrentBits;

    // calculate the number of bits to extract the desired number of bits
    ui_right_shift = m_iValidBits + uiNumberOfBits ;

    // mask out the value in the next word
    ruiBits |= m_uiNextBits >> ui_right_shift;

    ruiBits >>= 32 - uiNumberOfBits;

    m_uiNextBits <<=  -m_iValidBits;
  }

  // check the current word for being empty
  //-- baekeun.lee@samsung.com
  if ( 0 < m_iValidBits)
  {
    return ;
  }
  xReadNextWord();
}

Void TComBitstream::readAlignOne()
{
  UInt uiNumberOfBits = getBitsUntilByteAligned();

  // check the number_of_bits parameter matches the range
  assert (uiNumberOfBits <= 32);
  assert (uiNumberOfBits <= m_uiBitsLeft);

  // sub the desired number of bits
  m_uiBitsLeft -= uiNumberOfBits;
  m_iValidBits -= uiNumberOfBits;

  assert (m_uiBitsLeft%8 == 0);
  assert (m_iValidBits%8 == 0);

  // check the current word for beeing still valid
  if( 0 < m_iValidBits )
  {
    m_ulCurrentBits <<= uiNumberOfBits;
    return;
  }

  xReadNextWord();

  // shift to the right position
  m_ulCurrentBits <<= 32 - m_iValidBits;

  return;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

__inline Void TComBitstream::xReadNextWord()
{
  m_ulCurrentBits = m_uiNextBits;
  m_iValidBits += 32;

  // chech if there are bytes left in the packet
  if( m_uiDWordsLeft )
  {
    // read 32 bit from the packet
    m_uiNextBits = xSwap( *m_pulStreamPacket++ );
    m_uiDWordsLeft--;
  }
  else
  {
    Int iBytesLeft  = ((Int)m_uiBitsLeft - m_iValidBits+7) >> 3;
    UChar* puc      = (UChar*) m_pulStreamPacket;
    m_uiNextBits  = 0;

    if( iBytesLeft > 0)
    {
      for( Int iByte = 0; iByte < iBytesLeft; iByte++ )
      {
        m_uiNextBits <<= 8;
        m_uiNextBits += puc[iByte];
      }
      m_uiNextBits <<= (4-iBytesLeft)<<3;
    }
  }
}

#if HHI_NAL_UNIT_SYNTAX

Void TComBitstream::initParsingConvertPayloadToRBSP( const UInt uiBytesRead )
{
  UInt uiZeroCount    = 0;
  UInt uiReadOffset   = 0;
  UInt uiWriteOffset  = 0;
  const UChar* pucRead = reinterpret_cast<UChar*> (getBuffer());
  UChar* pucWrite      = reinterpret_cast<UChar*> (getBuffer());

  for( ; uiReadOffset < uiBytesRead; uiReadOffset++ )
  {
    if( 2 == uiZeroCount && 0x03 == pucRead[uiReadOffset] )
    {
      uiReadOffset++;
      uiZeroCount = 0;
      if (uiReadOffset>=uiBytesRead)
      {
        break;
      }
    }

    pucWrite[uiWriteOffset++] = pucRead[uiReadOffset];

    if( 0x00 == pucRead[uiReadOffset] )
    {
      uiZeroCount++;
    }
    else
    {
      uiZeroCount = 0;
    }
  }

  // th just clear the remaining bits in the buffer
  for( UInt ui = uiWriteOffset; ui < uiBytesRead; ui++)
  {
    pucWrite[ui] = 0;
  }

  initParsing( uiWriteOffset );
}

Void TComBitstream::convertRBSPToPayload( UInt uiStartPos )
{
  UInt uiZeroCount    = 0;

  //make sure the buffer is flushed
  assert( 0 == getBitsUntilByteAligned() );

  const UInt uiBytesInBuffer = getNumberOfWrittenBits()>>3;
  //make sure there's something in the buffer
  assert( 0 != uiBytesInBuffer );

  //make sure start pos is inside the buffer
//  assert( uiStartPos > uiBytesInBuffer );
  
  UChar* pucRead = new UChar[ uiBytesInBuffer ];
  //th this is not nice but ...
  memcpy( pucRead, getStartStream(), uiBytesInBuffer );

  UChar* pucWrite      =  reinterpret_cast<UChar*> (getStartStream());

  UInt uiWriteOffset  = uiStartPos;
  for( UInt uiReadOffset = uiStartPos; uiReadOffset < uiBytesInBuffer ; uiReadOffset++ )
  {
    if( 2 == uiZeroCount && 0 == (pucRead[uiReadOffset] & 0xfc) )
    {
      pucWrite[uiWriteOffset++] = 0x03;
      uiZeroCount = 0;
    }

    pucWrite[uiWriteOffset++] = pucRead[uiReadOffset];

    if( 0 == pucRead[uiReadOffset] )
    {
      uiZeroCount++;
    }
    else
    {
      uiZeroCount = 0;
    }
  }

  delete [] pucRead;
  m_uiBitsWritten = uiWriteOffset << 3;
}
#endif
>>>>>>> upstream/master
