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

/** \file     TComYuv.cpp
    \brief    general YUV buffer class
    \todo     this should be merged with TComPicYuv
*/

#include <stdlib.h>
#include <memory.h>
#include <assert.h>
#include <math.h>

#include "CommonDef.h"
#include "TComYuv.h"
#include "TComInterpolationFilter.h"

//! \ingroup TLibCommon
//! \{

TComYuv::TComYuv()
{
  for(Int comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    m_apiBuf[comp] = NULL;
  }
}

TComYuv::~TComYuv()
{
}

Void TComYuv::create( UInt iWidth, UInt iHeight, ChromaFormat chromaFormatIDC )
{
  // set width and height
  m_iWidth   = iWidth;
  m_iHeight  = iHeight;
  m_chromaFormatIDC = chromaFormatIDC;

  for(Int comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    // memory allocation
    m_apiBuf[comp]  = (Pel*)xMalloc( Pel, getWidth(ComponentID(comp))*getHeight(ComponentID(comp)) );
  }
}

Void TComYuv::destroy()
{
  // memory free
  for(Int comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    if (m_apiBuf[comp]!=NULL)
    {
      xFree( m_apiBuf[comp] );
      m_apiBuf[comp] = NULL;
    }
  }
}

Void TComYuv::clear()
{
  for(Int comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    if (m_apiBuf[comp]!=NULL)
    {
      ::memset( m_apiBuf[comp], 0, ( getWidth(ComponentID(comp)) * getHeight(ComponentID(comp))  )*sizeof(Pel) );
    }
  }
}




Void TComYuv::copyToPicYuv   ( TComPicYuv* pcPicYuvDst, const UInt ctuRsAddr, const UInt uiAbsZorderIdx, const UInt uiPartDepth, const UInt uiPartIdx ) const
{
  for(Int comp=0; comp<getNumberValidComponents(); comp++)
  {
    copyToPicComponent  ( ComponentID(comp), pcPicYuvDst, ctuRsAddr, uiAbsZorderIdx, uiPartDepth, uiPartIdx );
  }
}

Void TComYuv::copyToPicComponent  ( const ComponentID compID, TComPicYuv* pcPicYuvDst, const UInt ctuRsAddr, const UInt uiAbsZorderIdx, const UInt uiPartDepth, const UInt uiPartIdx ) const
{
  const Int iWidth  = getWidth(compID) >>uiPartDepth;
  const Int iHeight = getHeight(compID)>>uiPartDepth;

  const Pel* pSrc     = getAddr(compID, uiPartIdx, iWidth);
        Pel* pDst     = pcPicYuvDst->getAddr ( compID, ctuRsAddr, uiAbsZorderIdx );

  const UInt  iSrcStride  = getStride(compID);
  const UInt  iDstStride  = pcPicYuvDst->getStride(compID);

  for ( Int y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyFromPicYuv   ( const TComPicYuv* pcPicYuvSrc, const UInt ctuRsAddr, const UInt uiAbsZorderIdx )
{
  for(Int comp=0; comp<getNumberValidComponents(); comp++)
  {
    copyFromPicComponent  ( ComponentID(comp), pcPicYuvSrc, ctuRsAddr, uiAbsZorderIdx );
  }
}

Void TComYuv::copyFromPicComponent  ( const ComponentID compID, const TComPicYuv* pcPicYuvSrc, const UInt ctuRsAddr, const UInt uiAbsZorderIdx )
{
        Pel* pDst     = getAddr(compID);
  const Pel* pSrc     = pcPicYuvSrc->getAddr ( compID, ctuRsAddr, uiAbsZorderIdx );

  const UInt iDstStride  = getStride(compID);
  const UInt iSrcStride  = pcPicYuvSrc->getStride(compID);
  const Int  iWidth=getWidth(compID);
  const Int  iHeight=getHeight(compID);

  for (Int y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyToPartYuv( TComYuv* pcYuvDst, const UInt uiDstPartIdx ) const
{
  for(Int comp=0; comp<getNumberValidComponents(); comp++)
  {
    copyToPartComponent  ( ComponentID(comp), pcYuvDst, uiDstPartIdx );
  }
}

Void TComYuv::copyToPartComponent( const ComponentID compID, TComYuv* pcYuvDst, const UInt uiDstPartIdx ) const
{
  const Pel* pSrc     = getAddr(compID);
        Pel* pDst     = pcYuvDst->getAddr( compID, uiDstPartIdx );

  const UInt iSrcStride  = getStride(compID);
  const UInt iDstStride  = pcYuvDst->getStride(compID);
  const Int  iWidth=getWidth(compID);
  const Int  iHeight=getHeight(compID);

  for (Int y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyPartToYuv( TComYuv* pcYuvDst, const UInt uiSrcPartIdx ) const
{
  for(Int comp=0; comp<getNumberValidComponents(); comp++)
  {
    copyPartToComponent  ( ComponentID(comp), pcYuvDst, uiSrcPartIdx );
  }
}

Void TComYuv::copyPartToComponent( const ComponentID compID, TComYuv* pcYuvDst, const UInt uiSrcPartIdx ) const
{
  const Pel* pSrc     = getAddr(compID, uiSrcPartIdx);
        Pel* pDst     = pcYuvDst->getAddr(compID, 0 );

  const UInt  iSrcStride  = getStride(compID);
  const UInt  iDstStride  = pcYuvDst->getStride(compID);

  const UInt uiHeight = pcYuvDst->getHeight(compID);
  const UInt uiWidth = pcYuvDst->getWidth(compID);

  for ( UInt y = uiHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*uiWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyPartToPartYuv   ( TComYuv* pcYuvDst, const UInt uiPartIdx, const UInt iWidth, const UInt iHeight ) const
{
  for(Int comp=0; comp<getNumberValidComponents(); comp++)
  {
    copyPartToPartComponent   (ComponentID(comp), pcYuvDst, uiPartIdx, iWidth>>getComponentScaleX(ComponentID(comp)), iHeight>>getComponentScaleY(ComponentID(comp)) );
  }
}

Void TComYuv::copyPartToPartComponent  ( const ComponentID compID, TComYuv* pcYuvDst, const UInt uiPartIdx, const UInt iWidthComponent, const UInt iHeightComponent ) const
{
  const Pel* pSrc =           getAddr(compID, uiPartIdx);
        Pel* pDst = pcYuvDst->getAddr(compID, uiPartIdx);
  if( pSrc == pDst )
  {
    //th not a good idea
    //th best would be to fix the caller
    return ;
  }

  const UInt  iSrcStride = getStride(compID);
  const UInt  iDstStride = pcYuvDst->getStride(compID);
  for ( UInt y = iHeightComponent; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, iWidthComponent * sizeof(Pel) );
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
}




Void TComYuv::copyPartToPartComponentMxN  ( const ComponentID compID, TComYuv* pcYuvDst, const TComRectangle &rect) const
{
  const Pel* pSrc =           getAddrPix( compID, rect.x0, rect.y0 );
        Pel* pDst = pcYuvDst->getAddrPix( compID, rect.x0, rect.y0 );
  if( pSrc == pDst )
  {
    //th not a good idea
    //th best would be to fix the caller
    return ;
  }

  const UInt  iSrcStride = getStride(compID);
  const UInt  iDstStride = pcYuvDst->getStride(compID);
  const UInt uiHeightComponent=rect.height;
  const UInt uiWidthComponent=rect.width;
  for ( UInt y = uiHeightComponent; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, uiWidthComponent * sizeof( Pel ) );
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
}




Void TComYuv::addClip( const TComYuv* pcYuvSrc0, const TComYuv* pcYuvSrc1, const UInt uiTrUnitIdx, const UInt uiPartSize, const BitDepths &clipBitDepths )
{
  for(Int comp=0; comp<getNumberValidComponents(); comp++)
  {
    const ComponentID compID=ComponentID(comp);
    const Int uiPartWidth =uiPartSize>>getComponentScaleX(compID);
    const Int uiPartHeight=uiPartSize>>getComponentScaleY(compID);

    const Pel* pSrc0 = pcYuvSrc0->getAddr(compID, uiTrUnitIdx, uiPartWidth );
    const Pel* pSrc1 = pcYuvSrc1->getAddr(compID, uiTrUnitIdx, uiPartWidth );
          Pel* pDst  = getAddr(compID, uiTrUnitIdx, uiPartWidth );

    const UInt iSrc0Stride = pcYuvSrc0->getStride(compID);
    const UInt iSrc1Stride = pcYuvSrc1->getStride(compID);
    const UInt iDstStride  = getStride(compID);
    const Int clipbd = clipBitDepths.recon[toChannelType(compID)];
#if O0043_BEST_EFFORT_DECODING
    const Int bitDepthDelta = clipBitDepths.stream[toChannelType(compID)] - clipbd;
#endif

    for ( Int y = uiPartHeight-1; y >= 0; y-- )
    {
      for ( Int x = uiPartWidth-1; x >= 0; x-- )
      {
#if O0043_BEST_EFFORT_DECODING
        pDst[x] = Pel(ClipBD<Int>( Int(pSrc0[x]) + rightShiftEvenRounding<Pel>(pSrc1[x], bitDepthDelta), clipbd));
#else
        pDst[x] = Pel(ClipBD<Int>( Int(pSrc0[x]) + Int(pSrc1[x]), clipbd));
#endif
      }
      pSrc0 += iSrc0Stride;
      pSrc1 += iSrc1Stride;
      pDst  += iDstStride;
    }
  }
}




Void TComYuv::subtract( const TComYuv* pcYuvSrc0, const TComYuv* pcYuvSrc1, const UInt uiTrUnitIdx, const UInt uiPartSize )
{
  for(Int comp=0; comp<getNumberValidComponents(); comp++)
  {
    const ComponentID compID=ComponentID(comp);
    const Int uiPartWidth =uiPartSize>>getComponentScaleX(compID);
    const Int uiPartHeight=uiPartSize>>getComponentScaleY(compID);

    const Pel* pSrc0 = pcYuvSrc0->getAddr( compID, uiTrUnitIdx, uiPartWidth );
    const Pel* pSrc1 = pcYuvSrc1->getAddr( compID, uiTrUnitIdx, uiPartWidth );
          Pel* pDst  = getAddr( compID, uiTrUnitIdx, uiPartWidth );

    const Int  iSrc0Stride = pcYuvSrc0->getStride(compID);
    const Int  iSrc1Stride = pcYuvSrc1->getStride(compID);
    const Int  iDstStride  = getStride(compID);

    for (Int y = uiPartHeight-1; y >= 0; y-- )
    {
      for (Int x = uiPartWidth-1; x >= 0; x-- )
      {
        pDst[x] = pSrc0[x] - pSrc1[x];
      }
      pSrc0 += iSrc0Stride;
      pSrc1 += iSrc1Stride;
      pDst  += iDstStride;
    }
  }
}




Void TComYuv::addAvg( const TComYuv* pcYuvSrc0, const TComYuv* pcYuvSrc1, const UInt iPartUnitIdx, const UInt uiWidth, const UInt uiHeight, const BitDepths &clipBitDepths )
{
  for(Int comp=0; comp<getNumberValidComponents(); comp++)
  {
    const ComponentID compID=ComponentID(comp);
    const Pel* pSrc0  = pcYuvSrc0->getAddr( compID, iPartUnitIdx );
    const Pel* pSrc1  = pcYuvSrc1->getAddr( compID, iPartUnitIdx );
    Pel* pDst   = getAddr( compID, iPartUnitIdx );

    const UInt  iSrc0Stride = pcYuvSrc0->getStride(compID);
    const UInt  iSrc1Stride = pcYuvSrc1->getStride(compID);
    const UInt  iDstStride  = getStride(compID);
    const Int   clipbd      = clipBitDepths.recon[toChannelType(compID)];
    const Int   shiftNum    = std::max<Int>(2, (IF_INTERNAL_PREC - clipbd)) + 1;
    const Int   offset      = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;

    const Int   iWidth      = uiWidth  >> getComponentScaleX(compID);
    const Int   iHeight     = uiHeight >> getComponentScaleY(compID);

    if (iWidth&1)
    {
      assert(0);
      exit(-1);
    }
    else if (iWidth&2)
    {
      for ( Int y = 0; y < iHeight; y++ )
      {
        for (Int x=0 ; x < iWidth; x+=2 )
        {
          pDst[ x + 0 ] = ClipBD( rightShift(( pSrc0[ x + 0 ] + pSrc1[ x + 0 ] + offset ), shiftNum), clipbd );
          pDst[ x + 1 ] = ClipBD( rightShift(( pSrc0[ x + 1 ] + pSrc1[ x + 1 ] + offset ), shiftNum), clipbd );
        }
        pSrc0 += iSrc0Stride;
        pSrc1 += iSrc1Stride;
        pDst  += iDstStride;
      }
    }
    else
    {
      for ( Int y = 0; y < iHeight; y++ )
      {
        for (Int x=0 ; x < iWidth; x+=4 )
        {
          pDst[ x + 0 ] = ClipBD( rightShift(( pSrc0[ x + 0 ] + pSrc1[ x + 0 ] + offset ), shiftNum), clipbd );
          pDst[ x + 1 ] = ClipBD( rightShift(( pSrc0[ x + 1 ] + pSrc1[ x + 1 ] + offset ), shiftNum), clipbd );
          pDst[ x + 2 ] = ClipBD( rightShift(( pSrc0[ x + 2 ] + pSrc1[ x + 2 ] + offset ), shiftNum), clipbd );
          pDst[ x + 3 ] = ClipBD( rightShift(( pSrc0[ x + 3 ] + pSrc1[ x + 3 ] + offset ), shiftNum), clipbd );
        }
        pSrc0 += iSrc0Stride;
        pSrc1 += iSrc1Stride;
        pDst  += iDstStride;
      }
    }
  }
}

Void TComYuv::removeHighFreq( const TComYuv* pcYuvSrc,
                              const UInt uiPartIdx,
                              const UInt uiWidth,
                              const UInt uiHeight,
                              const Int bitDepths[MAX_NUM_CHANNEL_TYPE],
                              const Bool bClipToBitDepths
                              )
{
  for(Int comp=0; comp<getNumberValidComponents(); comp++)
  {
    const ComponentID compID=ComponentID(comp);
    const Pel* pSrc  = pcYuvSrc->getAddr(compID, uiPartIdx);
    Pel* pDst  = getAddr(compID, uiPartIdx);

    const Int iSrcStride = pcYuvSrc->getStride(compID);
    const Int iDstStride = getStride(compID);
    const Int iWidth  = uiWidth >>getComponentScaleX(compID);
    const Int iHeight = uiHeight>>getComponentScaleY(compID);
    if (bClipToBitDepths)
    {
      const Int clipBd=bitDepths[toChannelType(compID)];
      for ( Int y = iHeight-1; y >= 0; y-- )
      {
        for ( Int x = iWidth-1; x >= 0; x-- )
        {
          pDst[x ] = ClipBD((2 * pDst[x]) - pSrc[x], clipBd);
        }
        pSrc += iSrcStride;
        pDst += iDstStride;
      }
    }
    else
    {
      for ( Int y = iHeight-1; y >= 0; y-- )
      {
        for ( Int x = iWidth-1; x >= 0; x-- )
        {
          pDst[x ] = (2 * pDst[x]) - pSrc[x];
        }
        pSrc += iSrcStride;
        pDst += iDstStride;
      }
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

/** \file     TComYuv.cpp
    \brief    general YUV buffer class
    \todo     this should be merged with TComPicYuv
*/

#include <stdlib.h>
#include <memory.h>
#include <assert.h>
#include <math.h>

#include "CommonDef.h"
#include "TComYuv.h"

TComYuv::TComYuv()
{
  m_apiBufY = NULL;
  m_apiBufU = NULL;
  m_apiBufV = NULL;
}

TComYuv::~TComYuv()
{
}

Void TComYuv::printout()
{
  Int  x, y;

  Pel* pSrc = getLumaAddr(  );
  Int  iStride = getStride();


  printf("\nY ...");
  for ( y = 0; y < iStride; y++ )
  {
    printf ("\n");
    for ( x = 0; x < iStride; x++ )
    {
      printf ("%d ", pSrc[x]);
    }
    pSrc += iStride;
  }
}

Void TComYuv::create( UInt iWidth, UInt iHeight )
{
  // memory allocation
  m_apiBufY  = (Pel*)xMalloc( Pel, iWidth*iHeight    );
  m_apiBufU  = (Pel*)xMalloc( Pel, iWidth*iHeight >> 2 );
  m_apiBufV  = (Pel*)xMalloc( Pel, iWidth*iHeight >> 2 );

  // set width and height
  m_iWidth   = iWidth;
  m_iHeight  = iHeight;
  m_iCWidth  = iWidth  >> 1;
  m_iCHeight = iHeight >> 1;
}

Void TComYuv::destroy()
{
  // memory free
  xFree( m_apiBufY ); m_apiBufY = NULL;
  xFree( m_apiBufU ); m_apiBufU = NULL;
  xFree( m_apiBufV ); m_apiBufV = NULL;
}

Void TComYuv::clear()
{
  ::memset( m_apiBufY, 0, ( m_iWidth  * m_iHeight  )*sizeof(Pel) );
  ::memset( m_apiBufU, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) );
  ::memset( m_apiBufV, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) );
}

Void TComYuv::clearLuma()
{
  ::memset( m_apiBufY, 0, ( m_iWidth  * m_iHeight  )*sizeof(Pel) );
}

Void TComYuv::clearChromaUV(UInt UV)
{
  if( UV )
    ::memset( m_apiBufV, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) );
  else
    ::memset( m_apiBufU, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) );
}

Void TComYuv::clearChroma()
{
  ::memset( m_apiBufU, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) );
  ::memset( m_apiBufV, 0, ( m_iCWidth * m_iCHeight )*sizeof(Pel) );
}

Void TComYuv::copyToPicYuv   ( TComPicYuv* pcPicYuvDst, UInt iCuAddr, UInt uiAbsZorderIdx, UInt uiPartDepth, UInt uiPartIdx )
{
  copyToPicLuma  ( pcPicYuvDst, iCuAddr, uiAbsZorderIdx, uiPartDepth, uiPartIdx );
  copyToPicChroma( pcPicYuvDst, iCuAddr, uiAbsZorderIdx, uiPartDepth, uiPartIdx );
}

Void TComYuv::copyToPicLuma  ( TComPicYuv* pcPicYuvDst, UInt iCuAddr, UInt uiAbsZorderIdx, UInt uiPartDepth, UInt uiPartIdx )
{
  Int  y, iWidth, iHeight;
  iWidth  = m_iWidth >>uiPartDepth;
  iHeight = m_iHeight>>uiPartDepth;

  Pel* pSrc     = getLumaAddr(uiPartIdx, iWidth);
  Pel* pDst     = pcPicYuvDst->getLumaAddr ( iCuAddr, uiAbsZorderIdx );

  UInt  iSrcStride  = getStride();
  UInt  iDstStride  = pcPicYuvDst->getStride();

  for ( y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}

Void TComYuv::copyToPicChroma( TComPicYuv* pcPicYuvDst, UInt iCuAddr, UInt uiAbsZorderIdx, UInt uiPartDepth, UInt uiPartIdx )
{
  Int  y, iWidth, iHeight;
  iWidth  = m_iCWidth >>uiPartDepth;
  iHeight = m_iCHeight>>uiPartDepth;

  Pel* pSrcU      = getCbAddr(uiPartIdx, iWidth);
  Pel* pSrcV      = getCrAddr(uiPartIdx, iWidth);
  Pel* pDstU      = pcPicYuvDst->getCbAddr( iCuAddr, uiAbsZorderIdx );
  Pel* pDstV      = pcPicYuvDst->getCrAddr( iCuAddr, uiAbsZorderIdx );

  UInt  iSrcStride = getCStride();
  UInt  iDstStride = pcPicYuvDst->getCStride();
  for ( y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(iWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(iWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyFromPicYuv   ( TComPicYuv* pcPicYuvSrc, UInt iCuAddr, UInt uiAbsZorderIdx )
{
  copyFromPicLuma  ( pcPicYuvSrc, iCuAddr, uiAbsZorderIdx );
  copyFromPicChroma( pcPicYuvSrc, iCuAddr, uiAbsZorderIdx );
}

Void TComYuv::copyFromPicLuma  ( TComPicYuv* pcPicYuvSrc, UInt iCuAddr, UInt uiAbsZorderIdx )
{
  Int  y;

  Pel* pDst     = m_apiBufY;
  Pel* pSrc     = pcPicYuvSrc->getLumaAddr ( iCuAddr, uiAbsZorderIdx );

  UInt  iDstStride  = getStride();
  UInt  iSrcStride  = pcPicYuvSrc->getStride();
  for ( y = m_iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*m_iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}

Void TComYuv::copyFromPicChroma( TComPicYuv* pcPicYuvSrc, UInt iCuAddr, UInt uiAbsZorderIdx )
{
  Int  y;

  Pel* pDstU      = m_apiBufU;
  Pel* pDstV      = m_apiBufV;
  Pel* pSrcU      = pcPicYuvSrc->getCbAddr( iCuAddr, uiAbsZorderIdx );
  Pel* pSrcV      = pcPicYuvSrc->getCrAddr( iCuAddr, uiAbsZorderIdx );

  UInt  iDstStride = getCStride();
  UInt  iSrcStride = pcPicYuvSrc->getCStride();
  for ( y = m_iCHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(m_iCWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(m_iCWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyToPartYuv( TComYuv* pcYuvDst, UInt uiDstPartIdx )
{
  copyToPartLuma  ( pcYuvDst, uiDstPartIdx );
  copyToPartChroma( pcYuvDst, uiDstPartIdx );
}

Void TComYuv::copyToPartLuma( TComYuv* pcYuvDst, UInt uiDstPartIdx )
{
  Int  y;

  Pel* pSrc     = m_apiBufY;
  Pel* pDst     = pcYuvDst->getLumaAddr( uiDstPartIdx );

  UInt  iSrcStride  = getStride();
  UInt  iDstStride  = pcYuvDst->getStride();
  for ( y = m_iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*m_iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}

Void TComYuv::copyToPartChroma( TComYuv* pcYuvDst, UInt uiDstPartIdx )
{
  Int  y;

  Pel* pSrcU      = m_apiBufU;
  Pel* pSrcV      = m_apiBufV;
  Pel* pDstU      = pcYuvDst->getCbAddr( uiDstPartIdx );
  Pel* pDstV      = pcYuvDst->getCrAddr( uiDstPartIdx );

  UInt  iSrcStride = getCStride();
  UInt  iDstStride = pcYuvDst->getCStride();
  for ( y = m_iCHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(m_iCWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(m_iCWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyPartToYuv( TComYuv* pcYuvDst, UInt uiSrcPartIdx )
{
  copyPartToLuma  ( pcYuvDst, uiSrcPartIdx );
  copyPartToChroma( pcYuvDst, uiSrcPartIdx );
}

Void TComYuv::copyPartToLuma( TComYuv* pcYuvDst, UInt uiSrcPartIdx )
{
  Int  y;

  Pel* pSrc     = getLumaAddr(uiSrcPartIdx);
  Pel* pDst     = pcYuvDst->getLumaAddr( 0 );

  UInt  iSrcStride  = getStride();
  UInt  iDstStride  = pcYuvDst->getStride();

  UInt uiHeight = pcYuvDst->getHeight();
  UInt uiWidth = pcYuvDst->getWidth();

  for ( y = uiHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*uiWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}

Void TComYuv::copyPartToChroma( TComYuv* pcYuvDst, UInt uiSrcPartIdx )
{
  Int  y;

  Pel* pSrcU      = getCbAddr( uiSrcPartIdx );
  Pel* pSrcV      = getCrAddr( uiSrcPartIdx );
  Pel* pDstU      = pcYuvDst->getCbAddr( 0 );
  Pel* pDstV      = pcYuvDst->getCrAddr( 0 );

  UInt  iSrcStride = getCStride();
  UInt  iDstStride = pcYuvDst->getCStride();

  UInt uiCHeight = pcYuvDst->getCHeight();
  UInt uiCWidth = pcYuvDst->getCWidth();

  for ( y = uiCHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, sizeof(Pel)*(uiCWidth) );
    ::memcpy( pDstV, pSrcV, sizeof(Pel)*(uiCWidth) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyPartToPartYuv   ( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight )
{
  copyPartToPartLuma   (pcYuvDst, uiPartIdx, iWidth, iHeight );
  copyPartToPartChroma (pcYuvDst, uiPartIdx, iWidth>>1, iHeight>>1 );
}

Void TComYuv::copyPartToPartLuma  ( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight )
{
  Pel* pSrc =           getLumaAddr(uiPartIdx);
  Pel* pDst = pcYuvDst->getLumaAddr(uiPartIdx);
  if( pSrc == pDst )
  {
    //th not a good idea
    //th best would be to fix the caller 
    return ;
  }

  UInt  iSrcStride = getStride();
  UInt  iDstStride = pcYuvDst->getStride();
  for ( UInt y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, iWidth * sizeof(Pel) );
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
}

Void TComYuv::copyPartToPartChroma( TComYuv* pcYuvDst, UInt uiPartIdx, UInt iWidth, UInt iHeight )
{
  Pel*  pSrcU =           getCbAddr(uiPartIdx);
  Pel*  pSrcV =           getCrAddr(uiPartIdx);
  Pel*  pDstU = pcYuvDst->getCbAddr(uiPartIdx);
  Pel*  pDstV = pcYuvDst->getCrAddr(uiPartIdx);

  if( pSrcU == pDstU && pSrcV == pDstV)
  {
    //th not a good idea
    //th best would be to fix the caller 
    return ;
  }

  UInt   iSrcStride = getCStride();
  UInt   iDstStride = pcYuvDst->getCStride();
  for ( UInt y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDstU, pSrcU, iWidth * sizeof(Pel) );
    ::memcpy( pDstV, pSrcV, iWidth * sizeof(Pel) );
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::copyToLuma( TComYuv*    pcYuvDst )
{
  memcpy(pcYuvDst->getLumaAddr(), m_apiBufY, sizeof(Pel)*m_iWidth*m_iHeight);
}
Void TComYuv::copyToChroma( TComYuv*    pcYuvDst )
{
  memcpy(pcYuvDst->getCbAddr(), m_apiBufU, sizeof(Pel)*m_iCWidth*m_iCHeight);
  memcpy(pcYuvDst->getCrAddr(), m_apiBufV, sizeof(Pel)*m_iCWidth*m_iCHeight);
}
Void TComYuv::copyToYuv( TComYuv*    pcYuvDst )
{
  copyToLuma(pcYuvDst);
  copyToChroma(pcYuvDst);
}

Void TComYuv::addClip( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  addClipLuma   ( pcYuvSrc0, pcYuvSrc1, uiTrUnitIdx, uiPartSize     );
  addClipChroma ( pcYuvSrc0, pcYuvSrc1, uiTrUnitIdx, uiPartSize>>1  );
}

Void TComYuv::addClipLuma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;

  Pel* pSrc0 = pcYuvSrc0->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrc1 = pcYuvSrc1->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );

  UInt iSrc0Stride = pcYuvSrc0->getStride();
  UInt iSrc1Stride = pcYuvSrc1->getStride();
  UInt iDstStride  = getStride();
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDst[x] = xClip( pSrc0[x] + pSrc1[x] );
    }
    pSrc0 += iSrc0Stride;
    pSrc1 += iSrc1Stride;
    pDst  += iDstStride;
  }
}

Void TComYuv::addClipChroma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;

  Pel* pSrcU0 = pcYuvSrc0->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcU1 = pcYuvSrc1->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV0 = pcYuvSrc0->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV1 = pcYuvSrc1->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstU = getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstV = getCrAddr( uiTrUnitIdx, uiPartSize );

  UInt  iSrc0Stride = pcYuvSrc0->getCStride();
  UInt  iSrc1Stride = pcYuvSrc1->getCStride();
  UInt  iDstStride  = getCStride();
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDstU[x] = xClip( pSrcU0[x] + pSrcU1[x] );
      pDstV[x] = xClip( pSrcV0[x] + pSrcV1[x] );
    }

    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
}

Void TComYuv::subtract( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  subtractLuma  ( pcYuvSrc0, pcYuvSrc1,  uiTrUnitIdx, uiPartSize    );
  subtractChroma( pcYuvSrc0, pcYuvSrc1,  uiTrUnitIdx, uiPartSize>>1 );
}

Void TComYuv::subtractLuma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;

  Pel* pSrc0 = pcYuvSrc0->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrc1 = pcYuvSrc1->getLumaAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDst  = getLumaAddr( uiTrUnitIdx, uiPartSize );

  Int  iSrc0Stride = pcYuvSrc0->getStride();
  Int  iSrc1Stride = pcYuvSrc1->getStride();
  Int  iDstStride  = getStride();
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDst[x] = pSrc0[x] - pSrc1[x];
    }
    pSrc0 += iSrc0Stride;
    pSrc1 += iSrc1Stride;
    pDst  += iDstStride;
  }
}

Void TComYuv::subtractChroma( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt uiTrUnitIdx, UInt uiPartSize )
{
  Int x, y;

  Pel* pSrcU0 = pcYuvSrc0->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcU1 = pcYuvSrc1->getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV0 = pcYuvSrc0->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pSrcV1 = pcYuvSrc1->getCrAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstU  = getCbAddr( uiTrUnitIdx, uiPartSize );
  Pel* pDstV  = getCrAddr( uiTrUnitIdx, uiPartSize );

  Int  iSrc0Stride = pcYuvSrc0->getCStride();
  Int  iSrc1Stride = pcYuvSrc1->getCStride();
  Int  iDstStride  = getCStride();
  for ( y = uiPartSize-1; y >= 0; y-- )
  {
    for ( x = uiPartSize-1; x >= 0; x-- )
    {
      pDstU[x] = pSrcU0[x] - pSrcU1[x];
      pDstV[x] = pSrcV0[x] - pSrcV1[x];
    }
    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
}

#ifdef ROUNDING_CONTROL_BIPRED

Void TComYuv::addAvg( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt iPartUnitIdx, UInt iWidth, UInt iHeight, Bool bRound )
{
  Int x, y;

  Pel* pSrcY0  = pcYuvSrc0->getLumaAddr( iPartUnitIdx );
  Pel* pSrcU0  = pcYuvSrc0->getCbAddr  ( iPartUnitIdx );
  Pel* pSrcV0  = pcYuvSrc0->getCrAddr  ( iPartUnitIdx );

  Pel* pSrcY1  = pcYuvSrc1->getLumaAddr( iPartUnitIdx );
  Pel* pSrcU1  = pcYuvSrc1->getCbAddr  ( iPartUnitIdx );
  Pel* pSrcV1  = pcYuvSrc1->getCrAddr  ( iPartUnitIdx );

  Pel* pDstY   = getLumaAddr( iPartUnitIdx );
  Pel* pDstU   = getCbAddr  ( iPartUnitIdx );
  Pel* pDstV   = getCrAddr  ( iPartUnitIdx );

  UInt  iSrc0Stride = pcYuvSrc0->getStride();
  UInt  iSrc1Stride = pcYuvSrc1->getStride();
  UInt  iDstStride  = getStride();

  for ( y = iHeight-1; y >= 0; y-- )
  {
    for ( x = iWidth-1; x >= 0; )
    {
      // note: luma min width is 4
      pDstY[x] = (pSrcY0[x] + pSrcY1[x] + bRound) >> 1; x--;
      pDstY[x] = (pSrcY0[x] + pSrcY1[x] + bRound) >> 1; x--;
      pDstY[x] = (pSrcY0[x] + pSrcY1[x] + bRound) >> 1; x--;
      pDstY[x] = (pSrcY0[x] + pSrcY1[x] + bRound) >> 1; x--;
    }
    pSrcY0 += iSrc0Stride;
    pSrcY1 += iSrc1Stride;
    pDstY  += iDstStride;
  }

  iSrc0Stride = pcYuvSrc0->getCStride();
  iSrc1Stride = pcYuvSrc1->getCStride();
  iDstStride  = getCStride();

  iWidth  >>=1;
  iHeight >>=1;

  for ( y = iHeight-1; y >= 0; y-- )
  {
    for ( x = iWidth-1; x >= 0; )
    {
      // note: chroma min width is 2
      pDstU[x] = (pSrcU0[x] + pSrcU1[x] + bRound) >> 1;
      pDstV[x] = (pSrcV0[x] + pSrcV1[x] + bRound) >> 1; x--;
      pDstU[x] = (pSrcU0[x] + pSrcU1[x] + bRound) >> 1;
      pDstV[x] = (pSrcV0[x] + pSrcV1[x] + bRound) >> 1; x--;
    }

    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
}

#endif

Void TComYuv::addAvg( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, UInt iPartUnitIdx, UInt iWidth, UInt iHeight )
{
  Int x, y;

  Pel* pSrcY0  = pcYuvSrc0->getLumaAddr( iPartUnitIdx );
  Pel* pSrcU0  = pcYuvSrc0->getCbAddr  ( iPartUnitIdx );
  Pel* pSrcV0  = pcYuvSrc0->getCrAddr  ( iPartUnitIdx );

  Pel* pSrcY1  = pcYuvSrc1->getLumaAddr( iPartUnitIdx );
  Pel* pSrcU1  = pcYuvSrc1->getCbAddr  ( iPartUnitIdx );
  Pel* pSrcV1  = pcYuvSrc1->getCrAddr  ( iPartUnitIdx );

  Pel* pDstY   = getLumaAddr( iPartUnitIdx );
  Pel* pDstU   = getCbAddr  ( iPartUnitIdx );
  Pel* pDstV   = getCrAddr  ( iPartUnitIdx );

  UInt  iSrc0Stride = pcYuvSrc0->getStride();
  UInt  iSrc1Stride = pcYuvSrc1->getStride();
  UInt  iDstStride  = getStride();

  for ( y = iHeight-1; y >= 0; y-- )
  {
    for ( x = iWidth-1; x >= 0; )
    {
      // note: luma min width is 4
      pDstY[x] = (pSrcY0[x] + pSrcY1[x] + 1) >> 1; x--;
      pDstY[x] = (pSrcY0[x] + pSrcY1[x] + 1) >> 1; x--;
      pDstY[x] = (pSrcY0[x] + pSrcY1[x] + 1) >> 1; x--;
      pDstY[x] = (pSrcY0[x] + pSrcY1[x] + 1) >> 1; x--;
    }
    pSrcY0 += iSrc0Stride;
    pSrcY1 += iSrc1Stride;
    pDstY  += iDstStride;
  }

  iSrc0Stride = pcYuvSrc0->getCStride();
  iSrc1Stride = pcYuvSrc1->getCStride();
  iDstStride  = getCStride();

  iWidth  >>=1;
  iHeight >>=1;

  for ( y = iHeight-1; y >= 0; y-- )
  {
    for ( x = iWidth-1; x >= 0; )
    {
      // note: chroma min width is 2
      pDstU[x] = (pSrcU0[x] + pSrcU1[x] + 1) >> 1;
      pDstV[x] = (pSrcV0[x] + pSrcV1[x] + 1) >> 1; x--;
      pDstU[x] = (pSrcU0[x] + pSrcU1[x] + 1) >> 1;
      pDstV[x] = (pSrcV0[x] + pSrcV1[x] + 1) >> 1; x--;
    }

    pSrcU0 += iSrc0Stride;
    pSrcU1 += iSrc1Stride;
    pSrcV0 += iSrc0Stride;
    pSrcV1 += iSrc1Stride;
    pDstU  += iDstStride;
    pDstV  += iDstStride;
  }
}

Void TComYuv::removeHighFreq( TComYuv* pcYuvSrc, UInt uiWidht, UInt uiHeight )
{
  Int x, y;

  Pel* pSrc  = pcYuvSrc->getLumaAddr();
  Pel* pSrcU = pcYuvSrc->getCbAddr();
  Pel* pSrcV = pcYuvSrc->getCrAddr();

  Pel* pDst  = m_apiBufY;
  Pel* pDstU = m_apiBufU;
  Pel* pDstV = m_apiBufV;

  Int  iSrcStride = pcYuvSrc->getStride();
  Int  iDstStride = getStride();

  for ( y = uiHeight-1; y >= 0; y-- )
  {
    for ( x = uiWidht-1; x >= 0; x-- )
    {
      pDst[x ] = xClip( (pDst[x ]<<1) - pSrc[x ] );
    }
    pSrc += iSrcStride;
    pDst += iDstStride;
  }

  iSrcStride = pcYuvSrc->getCStride();
  iDstStride = getCStride();

  uiHeight >>= 1;
  uiWidht  >>= 1;

  for ( y = uiHeight-1; y >= 0; y-- )
  {
    for ( x = uiWidht-1; x >= 0; x-- )
    {
      pDstU[x ] = xClip( (pDstU[x ]<<1) - pSrcU[x ] );
      pDstV[x ] = xClip( (pDstV[x ]<<1) - pSrcV[x ] );
    }
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}

Void TComYuv::removeHighFreq( TComYuv* pcYuvSrc, UInt uiPartIdx, UInt uiWidht, UInt uiHeight )
{
  Int x, y;

  Pel* pSrc  = pcYuvSrc->getLumaAddr(uiPartIdx);
  Pel* pSrcU = pcYuvSrc->getCbAddr(uiPartIdx);
  Pel* pSrcV = pcYuvSrc->getCrAddr(uiPartIdx);

  Pel* pDst  = getLumaAddr(uiPartIdx);
  Pel* pDstU = getCbAddr(uiPartIdx);
  Pel* pDstV = getCrAddr(uiPartIdx);

  Int  iSrcStride = pcYuvSrc->getStride();
  Int  iDstStride = getStride();

  for ( y = uiHeight-1; y >= 0; y-- )
  {
    for ( x = uiWidht-1; x >= 0; x-- )
    {
      pDst[x ] = xClip( (pDst[x ]<<1) - pSrc[x ] );
    }
    pSrc += iSrcStride;
    pDst += iDstStride;
  }

  iSrcStride = pcYuvSrc->getCStride();
  iDstStride = getCStride();

  uiHeight >>= 1;
  uiWidht  >>= 1;

  for ( y = uiHeight-1; y >= 0; y-- )
  {
    for ( x = uiWidht-1; x >= 0; x-- )
    {
      pDstU[x ] = xClip( (pDstU[x ]<<1) - pSrcU[x ] );
      pDstV[x ] = xClip( (pDstV[x ]<<1) - pSrcV[x ] );
    }
    pSrcU += iSrcStride;
    pSrcV += iSrcStride;
    pDstU += iDstStride;
    pDstV += iDstStride;
  }
}


Pel* TComYuv::getLumaAddr( UInt uiPartUnitIdx )
{
  UInt iBlkX;
  UInt iBlkY;
  iBlkX = g_auiRasterToPelX[g_auiZscanToRaster[uiPartUnitIdx]];
  iBlkY = g_auiRasterToPelY[g_auiZscanToRaster[uiPartUnitIdx]];

  return m_apiBufY + iBlkY*getStride() + iBlkX;
}

Pel* TComYuv::getCbAddr( UInt uiPartUnitIdx )
{
  UInt iBlkX;
  UInt iBlkY;
  iBlkX = g_auiRasterToPelX[g_auiZscanToRaster[uiPartUnitIdx]] >> 1;
  iBlkY = g_auiRasterToPelY[g_auiZscanToRaster[uiPartUnitIdx]] >> 1;

  return m_apiBufU + iBlkY*getCStride() + iBlkX;
}

Pel* TComYuv::getCrAddr( UInt uiPartUnitIdx )
{
  UInt iBlkX;
  UInt iBlkY;
  iBlkX = g_auiRasterToPelX[g_auiZscanToRaster[uiPartUnitIdx]] >> 1;
  iBlkY = g_auiRasterToPelY[g_auiZscanToRaster[uiPartUnitIdx]] >> 1;

  return m_apiBufV + iBlkY*getCStride() + iBlkX;
}

Pel* TComYuv::getLumaAddr( UInt iTransUnitIdx, UInt iBlkSize )
{
  UInt uiNumTrInWidth = m_iWidth / iBlkSize;
  UInt   iBlkX   = iTransUnitIdx % uiNumTrInWidth;
  UInt   iBlkY   = iTransUnitIdx / uiNumTrInWidth;

  return m_apiBufY + (iBlkX + iBlkY * getStride()) * iBlkSize;
}

Pel* TComYuv::getCbAddr( UInt iTransUnitIdx, UInt iBlkSize )
{
  UInt uiNumTrInWidth = m_iCWidth / iBlkSize;
  UInt   iBlkX   = iTransUnitIdx % uiNumTrInWidth;
  UInt   iBlkY   = iTransUnitIdx / uiNumTrInWidth;

  return m_apiBufU + (iBlkX + iBlkY * getCStride()) * iBlkSize;
}

Pel* TComYuv::getCrAddr( UInt iTransUnitIdx, UInt iBlkSize )
{
  UInt uiNumTrInWidth = m_iCWidth / iBlkSize;
  UInt   iBlkX   = iTransUnitIdx % uiNumTrInWidth;
  UInt   iBlkY   = iTransUnitIdx / uiNumTrInWidth;

  return m_apiBufV + (iBlkX + iBlkY * getCStride()) * iBlkSize;
}

>>>>>>> upstream/master
