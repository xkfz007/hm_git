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

/** \file     TComMotionInfo.cpp
    \brief    motion information handling classes
*/

#include <memory.h>
#include "TComMotionInfo.h"
#include "assert.h"
#include <stdlib.h>

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// Create / destroy
// --------------------------------------------------------------------------------------------------------------------

Void TComCUMvField::create( UInt uiNumPartition )
{
  assert(m_pcMv     == NULL);
  assert(m_pcMvd    == NULL);
  assert(m_piRefIdx == NULL);

  m_pcMv     = new TComMv[ uiNumPartition ];
  m_pcMvd    = new TComMv[ uiNumPartition ];
  m_piRefIdx = new Char  [ uiNumPartition ];

  m_uiNumPartition = uiNumPartition;
}

Void TComCUMvField::destroy()
{
  assert(m_pcMv     != NULL);
  assert(m_pcMvd    != NULL);
  assert(m_piRefIdx != NULL);

  delete[] m_pcMv;
  delete[] m_pcMvd;
  delete[] m_piRefIdx;

  m_pcMv     = NULL;
  m_pcMvd    = NULL;
  m_piRefIdx = NULL;

  m_uiNumPartition = 0;
}

// --------------------------------------------------------------------------------------------------------------------
// Clear / copy
// --------------------------------------------------------------------------------------------------------------------

Void TComCUMvField::clearMvField()
{
  for ( Int i = 0; i < m_uiNumPartition; i++ )
  {
    m_pcMv [ i ].setZero();
    m_pcMvd[ i ].setZero();
  }
  assert( sizeof( *m_piRefIdx ) == 1 );
  memset( m_piRefIdx, NOT_VALID, m_uiNumPartition * sizeof( *m_piRefIdx ) );
}

Void TComCUMvField::copyFrom( TComCUMvField const * pcCUMvFieldSrc, Int iNumPartSrc, Int iPartAddrDst )
{
  Int iSizeInTComMv = sizeof( TComMv ) * iNumPartSrc;

  memcpy( m_pcMv     + iPartAddrDst, pcCUMvFieldSrc->m_pcMv,     iSizeInTComMv );
  memcpy( m_pcMvd    + iPartAddrDst, pcCUMvFieldSrc->m_pcMvd,    iSizeInTComMv );
  memcpy( m_piRefIdx + iPartAddrDst, pcCUMvFieldSrc->m_piRefIdx, sizeof( *m_piRefIdx ) * iNumPartSrc );
}

Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst ) const
{
  copyTo( pcCUMvFieldDst, iPartAddrDst, 0, m_uiNumPartition );
}

Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst, UInt uiOffset, UInt uiNumPart ) const
{
  Int iSizeInTComMv = sizeof( TComMv ) * uiNumPart;
  Int iOffset = uiOffset + iPartAddrDst;

  memcpy( pcCUMvFieldDst->m_pcMv     + iOffset, m_pcMv     + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->m_pcMvd    + iOffset, m_pcMvd    + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->m_piRefIdx + iOffset, m_piRefIdx + uiOffset, sizeof( *m_piRefIdx ) * uiNumPart );
}

// --------------------------------------------------------------------------------------------------------------------
// Set
// --------------------------------------------------------------------------------------------------------------------

template <typename T>
Void TComCUMvField::setAll( T *p, T const & val, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx  )
{
  Int i;
  p += iPartAddr;
  Int numElements = m_uiNumPartition >> ( 2 * uiDepth );

  switch( eCUMode )
  {
    case SIZE_2Nx2N:
      for ( i = 0; i < numElements; i++ )
      {
        p[ i ] = val;
      }
      break;

    case SIZE_2NxN:
      numElements >>= 1;
      for ( i = 0; i < numElements; i++ )
      {
        p[ i ] = val;
      }
      break;

    case SIZE_Nx2N:
      numElements >>= 2;
      for ( i = 0; i < numElements; i++ )
      {
        p[ i                   ] = val;
        p[ i + 2 * numElements ] = val;
      }
      break;

    case SIZE_NxN:
      numElements >>= 2;
      for ( i = 0; i < numElements; i++)
      {
        p[ i ] = val;
      }
      break;
    case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = numElements>>2;
      if( iPartIdx == 0 )
      {
        T *pT  = p;
        T *pT2 = p + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }
      }
      else
      {
        T *pT  = p;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pT[i] = val;
        }

        pT = p + iCurrPartNumQ;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pT[i] = val;
        }
      }
      break;
    }
  case SIZE_2NxnD:
    {
      Int iCurrPartNumQ = numElements>>2;
      if( iPartIdx == 0 )
      {
        T *pT  = p;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pT[i] = val;
        }
        pT = p + ( numElements - iCurrPartNumQ );
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pT[i] = val;
        }
      }
      else
      {
        T *pT  = p;
        T *pT2 = p + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = numElements>>2;
      if( iPartIdx == 0 )
      {
        T *pT  = p;
        T *pT2 = p + (iCurrPartNumQ<<1);
        T *pT3 = p + (iCurrPartNumQ>>1);
        T *pT4 = p + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pT [i] = val;
          pT2[i] = val;
          pT3[i] = val;
          pT4[i] = val;
        }
      }
      else
      {
        T *pT  = p;
        T *pT2 = p + (iCurrPartNumQ<<1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }

        pT  = p + (iCurrPartNumQ>>1);
        pT2 = p + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }
      }
      break;
    }
  case SIZE_nRx2N:
    {
      Int iCurrPartNumQ = numElements>>2;
      if( iPartIdx == 0 )
      {
        T *pT  = p;
        T *pT2 = p + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }

        pT  = p + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pT2 = p + numElements - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }
      }
      else
      {
        T *pT  = p;
        T *pT2 = p + (iCurrPartNumQ>>1);
        T *pT3 = p + (iCurrPartNumQ<<1);
        T *pT4 = p + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pT [i] = val;
          pT2[i] = val;
          pT3[i] = val;
          pT4[i] = val;
        }
      }
      break;
    }
    default:
      assert(0);
      break;
  }
}

Void TComCUMvField::setAllMv( TComMv const & mv, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
{
  setAll(m_pcMv, mv, eCUMode, iPartAddr, uiDepth, iPartIdx);
}

Void TComCUMvField::setAllMvd( TComMv const & mvd, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
{
  setAll(m_pcMvd, mvd, eCUMode, iPartAddr, uiDepth, iPartIdx);
}

Void TComCUMvField::setAllRefIdx ( Int iRefIdx, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
{
  setAll(m_piRefIdx, static_cast<Char>(iRefIdx), eCUMode, iPartAddr, uiDepth, iPartIdx);
}

Void TComCUMvField::setAllMvField( TComMvField const & mvField, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
{
  setAllMv    ( mvField.getMv(),     eCUMode, iPartAddr, uiDepth, iPartIdx );
  setAllRefIdx( mvField.getRefIdx(), eCUMode, iPartAddr, uiDepth, iPartIdx );
}

/**Subsampling of the stored prediction mode, reference index and motion vector
 * \param pePredMode Pointer to prediction modes
 * \param scale      Factor by which to subsample motion information
 */
Void TComCUMvField::compress(Char* pePredMode, Int scale)
{
  Int N = scale * scale;
  assert( N > 0 && N <= m_uiNumPartition);

  for ( Int uiPartIdx = 0; uiPartIdx < m_uiNumPartition; uiPartIdx += N )
  {
    TComMv cMv(0,0);
    Int iRefIdx = 0;

    cMv = m_pcMv[ uiPartIdx ];
    PredMode predMode = static_cast<PredMode>( pePredMode[ uiPartIdx ] );
    iRefIdx = m_piRefIdx[ uiPartIdx ];
    for ( Int i = 0; i < N; i++ )
    {
      m_pcMv[ uiPartIdx + i ] = cMv;
      pePredMode[ uiPartIdx + i ] = predMode;
      m_piRefIdx[ uiPartIdx + i ] = iRefIdx;
    }
  }
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

/** \file     TComMotionInfo.cpp
    \brief    motion information handling classes
*/

#include <memory.h>
#include "TComMotionInfo.h"
#include "assert.h"
#include <stdlib.h>

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// Create / destroy
// --------------------------------------------------------------------------------------------------------------------

Void TComCUMvField::create( UInt uiNumPartition )
{
  m_pcMv     = ( TComMv* )xMalloc( TComMv, uiNumPartition );
  m_pcMvd    = ( TComMv* )xMalloc( TComMv, uiNumPartition );
  m_piRefIdx = (    Int* )xMalloc( Int,    uiNumPartition );

#ifdef QC_AMVRES
  m_bMVRes     = (    Bool* )xMalloc( Bool,    uiNumPartition );
#endif

  m_uiNumPartition = uiNumPartition;
}

Void TComCUMvField::destroy()
{
  if( m_pcMv )
  {
    xFree( m_pcMv );     m_pcMv     = NULL;
  }
  if( m_pcMvd )
  {
    xFree( m_pcMvd );    m_pcMvd    = NULL;
  }
  if( m_piRefIdx )
  {
    xFree( m_piRefIdx ); m_piRefIdx = NULL;
  }
#ifdef QC_AMVRES
  if( m_bMVRes )
  {
    xFree( m_bMVRes ); m_bMVRes = NULL;
  }
#endif
}

// --------------------------------------------------------------------------------------------------------------------
// Clear / copy
// --------------------------------------------------------------------------------------------------------------------

Void TComCUMvField::clearMv( Int iPartAddr, UInt uiDepth )
{
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  for ( Int i = iNumPartition - 1; i >= 0; i-- )
  {
    m_pcMv[ i ].setZero();
  }
}

Void TComCUMvField::clearMvd( Int iPartAddr, UInt uiDepth )
{
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  for ( Int i = iNumPartition - 1; i >= 0; i-- )
  {
    m_pcMvd[ i ].setZero();
  }
}

Void TComCUMvField::clearMvField()
{
  for ( Int i = m_uiNumPartition - 1; i >= 0; i-- )
  {
    m_pcMv    [ i ].setZero();
    m_pcMvd   [ i ].setZero();
    m_piRefIdx[ i ] = NOT_VALID;
#ifdef QC_AMVRES
	m_bMVRes[ i ] = false;
#endif
  }
}

Void TComCUMvField::copyFrom( TComCUMvField* pcCUMvFieldSrc, Int iNumPartSrc, Int iPartAddrDst )
{
  Int iSizeInTComMv = sizeof( TComMv ) * iNumPartSrc;

  memcpy( m_pcMv     + iPartAddrDst, pcCUMvFieldSrc->getMv(),     iSizeInTComMv );
  memcpy( m_pcMvd    + iPartAddrDst, pcCUMvFieldSrc->getMvd(),    iSizeInTComMv );
  memcpy( m_piRefIdx + iPartAddrDst, pcCUMvFieldSrc->getRefIdx(), sizeof( Int ) * iNumPartSrc );
#ifdef QC_AMVRES
  memcpy( m_bMVRes + iPartAddrDst, pcCUMvFieldSrc->getMVRes(), sizeof( Bool ) * iNumPartSrc );
#endif

}

Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst )
{
  Int iSizeInTComMv = sizeof( TComMv ) * m_uiNumPartition;

  memcpy( pcCUMvFieldDst->getMv()     + iPartAddrDst, m_pcMv,     iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvd()    + iPartAddrDst, m_pcMvd,    iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getRefIdx() + iPartAddrDst, m_piRefIdx, sizeof( Int ) * m_uiNumPartition );
#ifdef QC_AMVRES
  memcpy( pcCUMvFieldDst->getMVRes()    + iPartAddrDst, m_bMVRes , sizeof( Bool ) * m_uiNumPartition );
#endif
}

Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst, UInt uiOffset, UInt uiNumPart )
{
  Int iSizeInTComMv = sizeof( TComMv ) * uiNumPart;
  Int iOffset = uiOffset + iPartAddrDst;

  memcpy( pcCUMvFieldDst->getMv()     + iOffset, m_pcMv     + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getMvd()    + iOffset, m_pcMvd    + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->getRefIdx() + iOffset, m_piRefIdx + uiOffset, sizeof( Int ) * uiNumPart );
#ifdef QC_AMVRES
  memcpy( pcCUMvFieldDst->getMVRes()+ iPartAddrDst,  m_bMVRes + uiOffset, sizeof( Bool ) * uiNumPart );
#endif
}

Void TComCUMvField::copyMvTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst )
{
  memcpy( pcCUMvFieldDst->getMv() + iPartAddrDst, m_pcMv, sizeof( TComMv ) * m_uiNumPartition );
}

// --------------------------------------------------------------------------------------------------------------------
// Set
// --------------------------------------------------------------------------------------------------------------------

Void TComCUMvField::setAllMv( TComMv& rcMv, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  Int i;
  TComMv* pcMv = m_pcMv + iPartAddr;
  register TComMv cMv = rcMv;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  switch( eCUMode )
  {
  case SIZE_2Nx2N:
    for ( i = iNumPartition - 1; i >= 0; i-- )
    {
      pcMv[ i ] = cMv;
    }
    break;
  case SIZE_2NxN:
    for ( i = ( iNumPartition >> 1 ) - 1; i >= 0; i-- )
    {
      pcMv[ i ] = cMv;
    }
    break;
  case SIZE_Nx2N:
    {
      UInt uiOffset = iNumPartition >> 1;
      for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
      {
        pcMv[ i ] = cMv;
        pcMv[ i + uiOffset ] = cMv;
      }
      break;
    }
  case SIZE_NxN:
    for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
    {
      pcMv[ i ] = cMv;
    }
    break;
  case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }
      }
      else
      {
        TComMv* pi  = pcMv;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = cMv;
        }

        pi = pcMv + iCurrPartNumQ;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = cMv;
        }
      }
      break;
    }
  case SIZE_2NxnD:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMv;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = cMv;
        }
        pi = pcMv + ( iNumPartition - iCurrPartNumQ );
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = cMv;
        }
      }
      else
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + (iCurrPartNumQ<<1);
        TComMv* pi3 = pcMv + (iCurrPartNumQ>>1);
        TComMv* pi4 = pcMv + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
          pi3[i] = cMv;
          pi4[i] = cMv;
        }
      }
      else
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + (iCurrPartNumQ<<1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }

        pi  = pcMv + (iCurrPartNumQ>>1);
        pi2 = pcMv + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }
      }
      break;
    }
  case SIZE_nRx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }

        pi  = pcMv + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pi2 = pcMv + iNumPartition - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
        }
      }
      else
      {
        TComMv* pi  = pcMv;
        TComMv* pi2 = pcMv + (iCurrPartNumQ>>1);
        TComMv* pi3 = pcMv + (iCurrPartNumQ<<1);
        TComMv* pi4 = pcMv + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMv;
          pi2[i] = cMv;
          pi3[i] = cMv;
          pi4[i] = cMv;
        }
      }
      break;
    }
  default:
    assert(0);
    break;
  }
}

Void TComCUMvField::setAllMvd( TComMv& rcMvd, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  Int i;
  TComMv* pcMvd = m_pcMvd + iPartAddr;
  register TComMv cMvd = rcMvd;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  switch( eCUMode )
  {
  case SIZE_2Nx2N:
    for ( i = iNumPartition - 1; i >= 0; i-- )
    {
      pcMvd[ i ] = cMvd;
    }
    break;
  case SIZE_2NxN:
    for ( i = ( iNumPartition >> 1 ) - 1; i >= 0; i-- )
    {
      pcMvd[ i ] = cMvd;
    }
    break;
  case SIZE_Nx2N:
    {
      UInt uiOffset = iNumPartition >> 1;
      for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
      {
        pcMvd[ i ] = cMvd;
        pcMvd[ i + uiOffset ] = cMvd;
      }
      break;
    }
  case SIZE_NxN:
    for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
    {
      pcMvd[ i ] = cMvd;
    }
    break;
  case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }
      }
      else
      {
        TComMv* pi  = pcMvd;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = cMvd;
        }

        pi = pcMvd + iCurrPartNumQ;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = cMvd;
        }
      }
      break;
    }
  case SIZE_2NxnD:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMvd;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = cMvd;
        }
        pi = pcMvd + ( iNumPartition - iCurrPartNumQ );
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = cMvd;
        }
      }
      else
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + (iCurrPartNumQ<<1);
        TComMv* pi3 = pcMvd + (iCurrPartNumQ>>1);
        TComMv* pi4 = pcMvd + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
          pi3[i] = cMvd;
          pi4[i] = cMvd;
        }
      }
      else
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + (iCurrPartNumQ<<1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }

        pi  = pcMvd + (iCurrPartNumQ>>1);
        pi2 = pcMvd + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }
      }
      break;
    }
  case SIZE_nRx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }

        pi  = pcMvd + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pi2 = pcMvd + iNumPartition - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
        }
      }
      else
      {
        TComMv* pi  = pcMvd;
        TComMv* pi2 = pcMvd + (iCurrPartNumQ>>1);
        TComMv* pi3 = pcMvd + (iCurrPartNumQ<<1);
        TComMv* pi4 = pcMvd + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = cMvd;
          pi2[i] = cMvd;
          pi3[i] = cMvd;
          pi4[i] = cMvd;
        }
      }
      break;
    }
  default:
    assert(0);
    break;
  }
}

Void TComCUMvField::setAllRefIdx ( Int iRefIdx, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  Int i;
  Int* piRefIdx = m_piRefIdx + iPartAddr;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  switch( eCUMode )
  {
  case SIZE_2Nx2N:
    for ( i = iNumPartition - 1; i >= 0; i-- )
    {
      piRefIdx[ i ] = iRefIdx;
    }
    break;
  case SIZE_2NxN:
    for ( i = ( iNumPartition >> 1 ) - 1; i >= 0; i-- )
    {
      piRefIdx[ i ] = iRefIdx;
    }
    break;
  case SIZE_Nx2N:
    {
      UInt uiOffset = iNumPartition >> 1;
      for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
      {
        piRefIdx[ i ] = iRefIdx;
        piRefIdx[ i + uiOffset ] = iRefIdx;
      }
      break;
    }
  case SIZE_NxN:
    for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
    {
      piRefIdx[ i ] = iRefIdx;
    }
    break;
  case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }
      }
      else
      {
        Int* pi  = piRefIdx;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = iRefIdx;
        }

        pi = piRefIdx + iCurrPartNumQ;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = iRefIdx;
        }
      }
      break;
    }
  case SIZE_2NxnD:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Int* pi  = piRefIdx;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = iRefIdx;
        }
        pi = piRefIdx + iNumPartition - iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = iRefIdx;
        }
      }
      else
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + (iCurrPartNumQ<<1);
        Int* pi3 = piRefIdx + (iCurrPartNumQ>>1);
        Int* pi4 = piRefIdx + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
          pi3[i] = iRefIdx;
          pi4[i] = iRefIdx;
        }
      }
      else
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + (iCurrPartNumQ<<1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }

        pi  = piRefIdx + (iCurrPartNumQ>>1);
        pi2 = piRefIdx + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }
      }
      break;
    }
  case SIZE_nRx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }

        pi  = piRefIdx + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pi2 = piRefIdx + iNumPartition - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
        }
      }
      else
      {
        Int* pi  = piRefIdx;
        Int* pi2 = piRefIdx + (iCurrPartNumQ>>1);
        Int* pi3 = piRefIdx + (iCurrPartNumQ<<1);
        Int* pi4 = piRefIdx + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = iRefIdx;
          pi2[i] = iRefIdx;
          pi3[i] = iRefIdx;
          pi4[i] = iRefIdx;
        }
      }
      break;
    }
  default:
    assert(0);
    break;
  }
}

Void TComCUMvField::setAllMvField ( TComMv& rcMv, Int iRefIdx, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  setAllMv( rcMv, eCUMode, iPartAddr, iPartIdx, uiDepth);
  setAllRefIdx(iRefIdx, eCUMode,iPartAddr,iPartIdx,uiDepth);
#ifdef QC_AMVRES
  setAllMVRes(false, eCUMode,iPartAddr,iPartIdx,uiDepth);
#endif
  return;
}


#ifdef QC_AMVRES
Void TComCUMvField::setAllMvField_AMVRes ( TComMv& rcMv, Int iRefIdx, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  setAllMv( rcMv, eCUMode, iPartAddr, iPartIdx, uiDepth);
  setAllRefIdx(iRefIdx, eCUMode,iPartAddr,iPartIdx,uiDepth);
  setAllMVRes(!rcMv.isHAM(), eCUMode,iPartAddr,iPartIdx,uiDepth);
  return;
}
Void TComCUMvField::setAllMVRes( Bool bMVRes, PartSize eCUMode, Int iPartAddr, Int iPartIdx, UInt uiDepth )
{
  Int i;
  Bool* pbMVRes = m_bMVRes + iPartAddr;
  Int iNumPartition = m_uiNumPartition >> (uiDepth<<1);

  switch( eCUMode )
  {
  case SIZE_2Nx2N:
    for ( i = iNumPartition - 1; i >= 0; i-- )
    {
      pbMVRes[ i ] = bMVRes;
    }
    break;
  case SIZE_2NxN:
    for ( i = ( iNumPartition >> 1 ) - 1; i >= 0; i-- )
    {
      pbMVRes[ i ] = bMVRes;
    }
    break;
  case SIZE_Nx2N:
    {
      UInt uiOffset = iNumPartition >> 1;
      for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
      {
        pbMVRes[ i ] = bMVRes;
        pbMVRes[ i + uiOffset ] = bMVRes;
      }
      break;
    }
  case SIZE_NxN:
    for ( i = ( iNumPartition >> 2 ) - 1; i >= 0; i-- )
    {
      pbMVRes[ i ] = bMVRes;
    }
    break;
  case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }
      }
      else
      {
        Bool* pi  = pbMVRes;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = bMVRes;
        }

        pi = pbMVRes + iCurrPartNumQ;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = bMVRes;
        }
      }
      break;
    }
  case SIZE_2NxnD:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Bool* pi  = pbMVRes;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pi[i] = bMVRes;
        }
        pi = pbMVRes + iNumPartition - iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi[i] = bMVRes;
        }
      }
      else
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + (iCurrPartNumQ<<1);
        Bool* pi3 = pbMVRes + (iCurrPartNumQ>>1);
        Bool* pi4 = pbMVRes + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
          pi3[i] = bMVRes;
          pi4[i] = bMVRes;
        }
      }
      else
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + (iCurrPartNumQ<<1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }

        pi  = pbMVRes + (iCurrPartNumQ>>1);
        pi2 = pbMVRes + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }
      }
      break;
    }
  case SIZE_nRx2N:
    {
      Int iCurrPartNumQ = iNumPartition>>2;
      if( iPartIdx == 0 )
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }

        pi  = pbMVRes + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pi2 = pbMVRes + iNumPartition - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
        }
      }
      else
      {
        Bool* pi  = pbMVRes;
        Bool* pi2 = pbMVRes + (iCurrPartNumQ>>1);
        Bool* pi3 = pbMVRes + (iCurrPartNumQ<<1);
        Bool* pi4 = pbMVRes + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pi [i] = bMVRes;
          pi2[i] = bMVRes;
          pi3[i] = bMVRes;
          pi4[i] = bMVRes;
        }
      }
      break;
    }
  default:
    assert(0);
    break;
  }
}


#endif

>>>>>>> upstream/master
