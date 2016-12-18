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

/** \file     TDecCAVLC.cpp
\brief    CAVLC decoder class
*/

#include "TDecCAVLC.h"
#include "SEIread.h"
#include "TDecSlice.h"
#include "TLibCommon/TComChromaFormat.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
#endif

//! \ingroup TLibDecoder
//! \{

#if ENC_DEC_TRACE

Void  xTraceVPSHeader ()
{
  fprintf( g_hTrace, "=========== Video Parameter Set     ===========\n" );
}

Void  xTraceSPSHeader ()
{
  fprintf( g_hTrace, "=========== Sequence Parameter Set  ===========\n" );
}

Void  xTracePPSHeader ()
{
  fprintf( g_hTrace, "=========== Picture Parameter Set  ===========\n");
}

Void  xTraceSliceHeader ()
{
  fprintf( g_hTrace, "=========== Slice ===========\n");
}

#endif

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TDecCavlc::TDecCavlc()
{
}

TDecCavlc::~TDecCavlc()
{

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecCavlc::parseShortTermRefPicSet( TComSPS* sps, TComReferencePictureSet* rps, Int idx )
{
  UInt code;
  UInt interRPSPred;
  if (idx > 0)
  {
    READ_FLAG(interRPSPred, "inter_ref_pic_set_prediction_flag");  rps->setInterRPSPrediction(interRPSPred);
  }
  else
  {
    interRPSPred = false;
    rps->setInterRPSPrediction(false);
  }

  if (interRPSPred)
  {
    UInt bit;
    if(idx == sps->getRPSList()->getNumberOfReferencePictureSets())
    {
      READ_UVLC(code, "delta_idx_minus1" ); // delta index of the Reference Picture Set used for prediction minus 1
    }
    else
    {
      code = 0;
    }
    assert(code <= idx-1); // delta_idx_minus1 shall not be larger than idx-1, otherwise we will predict from a negative row position that does not exist. When idx equals 0 there is no legal value and interRPSPred must be zero. See J0185-r2
    Int rIdx =  idx - 1 - code;
    assert (rIdx <= idx-1 && rIdx >= 0); // Made assert tighter; if rIdx = idx then prediction is done from itself. rIdx must belong to range 0, idx-1, inclusive, see J0185-r2
    TComReferencePictureSet*   rpsRef = sps->getRPSList()->getReferencePictureSet(rIdx);
    Int k = 0, k0 = 0, k1 = 0;
    READ_CODE(1, bit, "delta_rps_sign"); // delta_RPS_sign
    READ_UVLC(code, "abs_delta_rps_minus1");  // absolute delta RPS minus 1
    Int deltaRPS = (1 - 2 * bit) * (code + 1); // delta_RPS
    for(Int j=0 ; j <= rpsRef->getNumberOfPictures(); j++)
    {
      READ_CODE(1, bit, "used_by_curr_pic_flag" ); //first bit is "1" if Idc is 1
      Int refIdc = bit;
      if (refIdc == 0)
      {
        READ_CODE(1, bit, "use_delta_flag" ); //second bit is "1" if Idc is 2, "0" otherwise.
        refIdc = bit<<1; //second bit is "1" if refIdc is 2, "0" if refIdc = 0.
      }
      if (refIdc == 1 || refIdc == 2)
      {
        Int deltaPOC = deltaRPS + ((j < rpsRef->getNumberOfPictures())? rpsRef->getDeltaPOC(j) : 0);
        rps->setDeltaPOC(k, deltaPOC);
        rps->setUsed(k, (refIdc == 1));

        if (deltaPOC < 0)
        {
          k0++;
        }
        else
        {
          k1++;
        }
        k++;
      }
      rps->setRefIdc(j,refIdc);
    }
    rps->setNumRefIdc(rpsRef->getNumberOfPictures()+1);
    rps->setNumberOfPictures(k);
    rps->setNumberOfNegativePictures(k0);
    rps->setNumberOfPositivePictures(k1);
    rps->sortDeltaPOC();
  }
  else
  {
    READ_UVLC(code, "num_negative_pics");           rps->setNumberOfNegativePictures(code);
    READ_UVLC(code, "num_positive_pics");           rps->setNumberOfPositivePictures(code);
    Int prev = 0;
    Int poc;
    for(Int j=0 ; j < rps->getNumberOfNegativePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s0_minus1");
      poc = prev-code-1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s0_flag");  rps->setUsed(j,code);
    }
    prev = 0;
    for(Int j=rps->getNumberOfNegativePictures(); j < rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s1_minus1");
      poc = prev+code+1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s1_flag");  rps->setUsed(j,code);
    }
    rps->setNumberOfPictures(rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures());
  }
#if PRINT_RPS_INFO
  rps->printDeltaPOC();
#endif
}

Void TDecCavlc::parsePPS(TComPPS* pcPPS)
{
#if ENC_DEC_TRACE
  xTracePPSHeader ();
#endif
  UInt  uiCode;

  Int   iCode;

  READ_UVLC( uiCode, "pps_pic_parameter_set_id");
  assert(uiCode <= 63);
  pcPPS->setPPSId (uiCode);

  READ_UVLC( uiCode, "pps_seq_parameter_set_id");
  assert(uiCode <= 15);
  pcPPS->setSPSId (uiCode);

  READ_FLAG( uiCode, "dependent_slice_segments_enabled_flag"    );    pcPPS->setDependentSliceSegmentsEnabledFlag   ( uiCode == 1 );

  READ_FLAG( uiCode, "output_flag_present_flag" );                    pcPPS->setOutputFlagPresentFlag( uiCode==1 );

  READ_CODE(3, uiCode, "num_extra_slice_header_bits");                pcPPS->setNumExtraSliceHeaderBits(uiCode);

  READ_FLAG ( uiCode, "sign_data_hiding_flag" ); pcPPS->setSignHideFlag( uiCode );

  READ_FLAG( uiCode,   "cabac_init_present_flag" );            pcPPS->setCabacInitPresentFlag( uiCode ? true : false );

  READ_UVLC(uiCode, "num_ref_idx_l0_default_active_minus1");
  assert(uiCode <= 14);
  pcPPS->setNumRefIdxL0DefaultActive(uiCode+1);

  READ_UVLC(uiCode, "num_ref_idx_l1_default_active_minus1");
  assert(uiCode <= 14);
  pcPPS->setNumRefIdxL1DefaultActive(uiCode+1);

  READ_SVLC(iCode, "init_qp_minus26" );                            pcPPS->setPicInitQPMinus26(iCode);
  READ_FLAG( uiCode, "constrained_intra_pred_flag" );              pcPPS->setConstrainedIntraPred( uiCode ? true : false );
  READ_FLAG( uiCode, "transform_skip_enabled_flag" );
  pcPPS->setUseTransformSkip ( uiCode ? true : false );

  READ_FLAG( uiCode, "cu_qp_delta_enabled_flag" );            pcPPS->setUseDQP( uiCode ? true : false );
  if( pcPPS->getUseDQP() )
  {
    READ_UVLC( uiCode, "diff_cu_qp_delta_depth" );
    pcPPS->setMaxCuDQPDepth( uiCode );
  }
  else
  {
    pcPPS->setMaxCuDQPDepth( 0 );
  }
  READ_SVLC( iCode, "pps_cb_qp_offset");
  pcPPS->setQpOffset(COMPONENT_Cb, iCode);
  assert( pcPPS->getQpOffset(COMPONENT_Cb) >= -12 );
  assert( pcPPS->getQpOffset(COMPONENT_Cb) <=  12 );

  READ_SVLC( iCode, "pps_cr_qp_offset");
  pcPPS->setQpOffset(COMPONENT_Cr, iCode);
  assert( pcPPS->getQpOffset(COMPONENT_Cr) >= -12 );
  assert( pcPPS->getQpOffset(COMPONENT_Cr) <=  12 );

  assert(MAX_NUM_COMPONENT<=3);

  READ_FLAG( uiCode, "pps_slice_chroma_qp_offsets_present_flag" );
  pcPPS->setSliceChromaQpFlag( uiCode ? true : false );

  READ_FLAG( uiCode, "weighted_pred_flag" );          // Use of Weighting Prediction (P_SLICE)
  pcPPS->setUseWP( uiCode==1 );
  READ_FLAG( uiCode, "weighted_bipred_flag" );         // Use of Bi-Directional Weighting Prediction (B_SLICE)
  pcPPS->setWPBiPred( uiCode==1 );

  READ_FLAG( uiCode, "transquant_bypass_enable_flag");
  pcPPS->setTransquantBypassEnableFlag(uiCode ? true : false);
  READ_FLAG( uiCode, "tiles_enabled_flag"               );    pcPPS->setTilesEnabledFlag            ( uiCode == 1 );
  READ_FLAG( uiCode, "entropy_coding_sync_enabled_flag" );    pcPPS->setEntropyCodingSyncEnabledFlag( uiCode == 1 );

  if( pcPPS->getTilesEnabledFlag() )
  {
    READ_UVLC ( uiCode, "num_tile_columns_minus1" );                pcPPS->setNumTileColumnsMinus1( uiCode );  
    READ_UVLC ( uiCode, "num_tile_rows_minus1" );                   pcPPS->setNumTileRowsMinus1( uiCode );  
    READ_FLAG ( uiCode, "uniform_spacing_flag" );                   pcPPS->setTileUniformSpacingFlag( uiCode == 1 );

    const UInt tileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
    const UInt tileRowsMinus1    = pcPPS->getNumTileRowsMinus1();
 
    if ( !pcPPS->getTileUniformSpacingFlag())
    {
      if (tileColumnsMinus1 > 0)
      {
        std::vector<Int> columnWidth(tileColumnsMinus1);
        for(UInt i = 0; i < tileColumnsMinus1; i++)
        { 
          READ_UVLC( uiCode, "column_width_minus1" );  
          columnWidth[i] = uiCode+1;
        }
        pcPPS->setTileColumnWidth(columnWidth);
      }

      if (tileRowsMinus1 > 0)
      {
        std::vector<Int> rowHeight (tileRowsMinus1);
        for(UInt i = 0; i < tileRowsMinus1; i++)
        {
          READ_UVLC( uiCode, "row_height_minus1" );
          rowHeight[i] = uiCode + 1;
        }
        pcPPS->setTileRowHeight(rowHeight);
      }
    }

    if ((tileColumnsMinus1 + tileRowsMinus1) != 0)
    {
      READ_FLAG ( uiCode, "loop_filter_across_tiles_enabled_flag" );   pcPPS->setLoopFilterAcrossTilesEnabledFlag( uiCode ? true : false );
    }
  }
  READ_FLAG( uiCode, "pps_loop_filter_across_slices_enabled_flag" );   pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode ? true : false );
  READ_FLAG( uiCode, "deblocking_filter_control_present_flag" );       pcPPS->setDeblockingFilterControlPresentFlag( uiCode ? true : false );
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    READ_FLAG( uiCode, "deblocking_filter_override_enabled_flag" );    pcPPS->setDeblockingFilterOverrideEnabledFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_disable_deblocking_filter_flag" );         pcPPS->setPicDisableDeblockingFilterFlag(uiCode ? true : false );
    if(!pcPPS->getPicDisableDeblockingFilterFlag())
    {
      READ_SVLC ( iCode, "pps_beta_offset_div2" );                     pcPPS->setDeblockingFilterBetaOffsetDiv2( iCode );
      READ_SVLC ( iCode, "pps_tc_offset_div2" );                       pcPPS->setDeblockingFilterTcOffsetDiv2( iCode );
    }
  }
  READ_FLAG( uiCode, "pps_scaling_list_data_present_flag" );           pcPPS->setScalingListPresentFlag( uiCode ? true : false );
  if(pcPPS->getScalingListPresentFlag ())
  {
    parseScalingList( &(pcPPS->getScalingList()) );
  }

  READ_FLAG( uiCode, "lists_modification_present_flag");
  pcPPS->setListsModificationPresentFlag(uiCode);

  READ_UVLC( uiCode, "log2_parallel_merge_level_minus2");
  pcPPS->setLog2ParallelMergeLevelMinus2 (uiCode);

  READ_FLAG( uiCode, "slice_segment_header_extension_present_flag");
  pcPPS->setSliceHeaderExtensionPresentFlag(uiCode);

  READ_FLAG( uiCode, "pps_extension_present_flag");
  if (uiCode)
  {
#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const char *syntaxStrings[]={ "pps_range_extension_flag",
                                         "pps_multilayer_extension_flag",
                                         "pps_extension_6bits[0]",
                                         "pps_extension_6bits[1]",
                                         "pps_extension_6bits[2]",
                                         "pps_extension_6bits[3]",
                                         "pps_extension_6bits[4]",
                                         "pps_extension_6bits[5]" };
#endif

    Bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS];
    for(Int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      pps_extension_flags[i] = uiCode!=0;
    }

    Bool bSkipTrailingExtensionBits=false;
    for(Int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (pps_extension_flags[i])
      {
        switch (PPSExtensionFlagIndex(i))
        {
          case PPS_EXT__REXT:
            {
              TComPPSRExt &ppsRangeExtension = pcPPS->getPpsRangeExtension();
              assert(!bSkipTrailingExtensionBits);

              if (pcPPS->getUseTransformSkip())
              {
                READ_UVLC( uiCode, "log2_max_transform_skip_block_size_minus2");
                ppsRangeExtension.setLog2MaxTransformSkipBlockSize(uiCode+2);
              }

              READ_FLAG( uiCode, "cross_component_prediction_enabled_flag");
              ppsRangeExtension.setCrossComponentPredictionEnabledFlag(uiCode != 0);

              READ_FLAG( uiCode, "chroma_qp_offset_list_enabled_flag");
              if (uiCode == 0)
              {
                ppsRangeExtension.clearChromaQpOffsetList();
                ppsRangeExtension.setDiffCuChromaQpOffsetDepth(0);
              }
              else
              {
                READ_UVLC(uiCode, "diff_cu_chroma_qp_offset_depth"); ppsRangeExtension.setDiffCuChromaQpOffsetDepth(uiCode);
                UInt tableSizeMinus1 = 0;
                READ_UVLC(tableSizeMinus1, "chroma_qp_offset_list_len_minus1");
                assert(tableSizeMinus1 < MAX_QP_OFFSET_LIST_SIZE);

                for (Int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= (tableSizeMinus1); cuChromaQpOffsetIdx++)
                {
                  Int cbOffset;
                  Int crOffset;
                  READ_SVLC(cbOffset, "cb_qp_offset_list[i]");
                  assert(cbOffset >= -12 && cbOffset <= 12);
                  READ_SVLC(crOffset, "cr_qp_offset_list[i]");
                  assert(crOffset >= -12 && crOffset <= 12);
                  // table uses +1 for index (see comment inside the function)
                  ppsRangeExtension.setChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1, cbOffset, crOffset);
                }
                assert(ppsRangeExtension.getChromaQpOffsetListLen() == tableSizeMinus1 + 1);
              }

              READ_UVLC( uiCode, "log2_sao_offset_scale_luma");
              ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_LUMA, uiCode);
              READ_UVLC( uiCode, "log2_sao_offset_scale_chroma");
              ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA, uiCode);
            }
            break;
          default:
            bSkipTrailingExtensionBits=true;
            break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "pps_extension_data_flag");
      }
    }
  }
  xReadRbspTrailingBits();
}

Void  TDecCavlc::parseVUI(TComVUI* pcVUI, TComSPS *pcSPS)
{
#if ENC_DEC_TRACE
  fprintf( g_hTrace, "----------- vui_parameters -----------\n");
#endif
  UInt  uiCode;

  READ_FLAG(     uiCode, "aspect_ratio_info_present_flag");           pcVUI->setAspectRatioInfoPresentFlag(uiCode);
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    READ_CODE(8, uiCode, "aspect_ratio_idc");                         pcVUI->setAspectRatioIdc(uiCode);
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      READ_CODE(16, uiCode, "sar_width");                             pcVUI->setSarWidth(uiCode);
      READ_CODE(16, uiCode, "sar_height");                            pcVUI->setSarHeight(uiCode);
    }
  }

  READ_FLAG(     uiCode, "overscan_info_present_flag");               pcVUI->setOverscanInfoPresentFlag(uiCode);
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    READ_FLAG(   uiCode, "overscan_appropriate_flag");                pcVUI->setOverscanAppropriateFlag(uiCode);
  }

  READ_FLAG(     uiCode, "video_signal_type_present_flag");           pcVUI->setVideoSignalTypePresentFlag(uiCode);
  if (pcVUI->getVideoSignalTypePresentFlag())
  {
    READ_CODE(3, uiCode, "video_format");                             pcVUI->setVideoFormat(uiCode);
    READ_FLAG(   uiCode, "video_full_range_flag");                    pcVUI->setVideoFullRangeFlag(uiCode);
    READ_FLAG(   uiCode, "colour_description_present_flag");          pcVUI->setColourDescriptionPresentFlag(uiCode);
    if (pcVUI->getColourDescriptionPresentFlag())
    {
      READ_CODE(8, uiCode, "colour_primaries");                       pcVUI->setColourPrimaries(uiCode);
      READ_CODE(8, uiCode, "transfer_characteristics");               pcVUI->setTransferCharacteristics(uiCode);
      READ_CODE(8, uiCode, "matrix_coeffs");                          pcVUI->setMatrixCoefficients(uiCode);
    }
  }

  READ_FLAG(     uiCode, "chroma_loc_info_present_flag");             pcVUI->setChromaLocInfoPresentFlag(uiCode);
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    READ_UVLC(   uiCode, "chroma_sample_loc_type_top_field" );        pcVUI->setChromaSampleLocTypeTopField(uiCode);
    READ_UVLC(   uiCode, "chroma_sample_loc_type_bottom_field" );     pcVUI->setChromaSampleLocTypeBottomField(uiCode);
  }

  READ_FLAG(     uiCode, "neutral_chroma_indication_flag");           pcVUI->setNeutralChromaIndicationFlag(uiCode);

  READ_FLAG(     uiCode, "field_seq_flag");                           pcVUI->setFieldSeqFlag(uiCode);

  READ_FLAG(uiCode, "frame_field_info_present_flag");                 pcVUI->setFrameFieldInfoPresentFlag(uiCode);

  READ_FLAG(     uiCode, "default_display_window_flag");
  if (uiCode != 0)
  {
    Window &defDisp = pcVUI->getDefaultDisplayWindow();
    READ_UVLC(   uiCode, "def_disp_win_left_offset" );                defDisp.setWindowLeftOffset  ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_right_offset" );               defDisp.setWindowRightOffset ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_top_offset" );                 defDisp.setWindowTopOffset   ( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_bottom_offset" );              defDisp.setWindowBottomOffset( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
  }

  TimingInfo *timingInfo = pcVUI->getTimingInfo();
  READ_FLAG(       uiCode, "vui_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vui_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vui_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vui_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vui_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }

    READ_FLAG(     uiCode, "vui_hrd_parameters_present_flag");        pcVUI->setHrdParametersPresentFlag(uiCode);
    if( pcVUI->getHrdParametersPresentFlag() )
    {
      parseHrdParameters( pcVUI->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
  }

  READ_FLAG(     uiCode, "bitstream_restriction_flag");               pcVUI->setBitstreamRestrictionFlag(uiCode);
  if (pcVUI->getBitstreamRestrictionFlag())
  {
    READ_FLAG(   uiCode, "tiles_fixed_structure_flag");               pcVUI->setTilesFixedStructureFlag(uiCode);
    READ_FLAG(   uiCode, "motion_vectors_over_pic_boundaries_flag");  pcVUI->setMotionVectorsOverPicBoundariesFlag(uiCode);
    READ_FLAG(   uiCode, "restricted_ref_pic_lists_flag");            pcVUI->setRestrictedRefPicListsFlag(uiCode);
    READ_UVLC(   uiCode, "min_spatial_segmentation_idc");             pcVUI->setMinSpatialSegmentationIdc(uiCode);
    assert(uiCode < 4096);
    READ_UVLC(   uiCode, "max_bytes_per_pic_denom" );                 pcVUI->setMaxBytesPerPicDenom(uiCode);
    READ_UVLC(   uiCode, "max_bits_per_min_cu_denom" );               pcVUI->setMaxBitsPerMinCuDenom(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_horizontal" );           pcVUI->setLog2MaxMvLengthHorizontal(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_vertical" );             pcVUI->setLog2MaxMvLengthVertical(uiCode);
  }
}

Void TDecCavlc::parseHrdParameters(TComHRD *hrd, Bool commonInfPresentFlag, UInt maxNumSubLayersMinus1)
{
  UInt  uiCode;
  if( commonInfPresentFlag )
  {
    READ_FLAG( uiCode, "nal_hrd_parameters_present_flag" );           hrd->setNalHrdParametersPresentFlag( uiCode == 1 ? true : false );
    READ_FLAG( uiCode, "vcl_hrd_parameters_present_flag" );           hrd->setVclHrdParametersPresentFlag( uiCode == 1 ? true : false );
    if( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() )
    {
      READ_FLAG( uiCode, "sub_pic_hrd_params_present_flag" );         hrd->setSubPicCpbParamsPresentFlag( uiCode == 1 ? true : false );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 8, uiCode, "tick_divisor_minus2" );                hrd->setTickDivisorMinus2( uiCode );
        READ_CODE( 5, uiCode, "du_cpb_removal_delay_increment_length_minus1" ); hrd->setDuCpbRemovalDelayLengthMinus1( uiCode );
        READ_FLAG( uiCode, "sub_pic_cpb_params_in_pic_timing_sei_flag" ); hrd->setSubPicCpbParamsInPicTimingSEIFlag( uiCode == 1 ? true : false );
        READ_CODE( 5, uiCode, "dpb_output_delay_du_length_minus1"  ); hrd->setDpbOutputDelayDuLengthMinus1( uiCode );
      }
      READ_CODE( 4, uiCode, "bit_rate_scale" );                       hrd->setBitRateScale( uiCode );
      READ_CODE( 4, uiCode, "cpb_size_scale" );                       hrd->setCpbSizeScale( uiCode );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 4, uiCode, "cpb_size_du_scale" );                  hrd->setDuCpbSizeScale( uiCode );
      }
      READ_CODE( 5, uiCode, "initial_cpb_removal_delay_length_minus1" ); hrd->setInitialCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "au_cpb_removal_delay_length_minus1" );      hrd->setCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "dpb_output_delay_length_minus1" );       hrd->setDpbOutputDelayLengthMinus1( uiCode );
    }
  }
  Int i, j, nalOrVcl;
  for( i = 0; i <= maxNumSubLayersMinus1; i ++ )
  {
    READ_FLAG( uiCode, "fixed_pic_rate_general_flag" );                     hrd->setFixedPicRateFlag( i, uiCode == 1 ? true : false  );
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      READ_FLAG( uiCode, "fixed_pic_rate_within_cvs_flag" );                hrd->setFixedPicRateWithinCvsFlag( i, uiCode == 1 ? true : false  );
    }
    else
    {
      hrd->setFixedPicRateWithinCvsFlag( i, true );
    }

    hrd->setLowDelayHrdFlag( i, 0 ); // Infered to be 0 when not present
    hrd->setCpbCntMinus1   ( i, 0 ); // Infered to be 0 when not present

    if( hrd->getFixedPicRateWithinCvsFlag( i ) )
    {
      READ_UVLC( uiCode, "elemental_duration_in_tc_minus1" );             hrd->setPicDurationInTcMinus1( i, uiCode );
    }
    else
    {
      READ_FLAG( uiCode, "low_delay_hrd_flag" );                      hrd->setLowDelayHrdFlag( i, uiCode == 1 ? true : false  );
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      READ_UVLC( uiCode, "cpb_cnt_minus1" );                          hrd->setCpbCntMinus1( i, uiCode );
    }

    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          READ_UVLC( uiCode, "bit_rate_value_minus1" );             hrd->setBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          READ_UVLC( uiCode, "cpb_size_value_minus1" );             hrd->setCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
          if( hrd->getSubPicCpbParamsPresentFlag() )
          {
            READ_UVLC( uiCode, "cpb_size_du_value_minus1" );       hrd->setDuCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
            READ_UVLC( uiCode, "bit_rate_du_value_minus1" );       hrd->setDuBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          }
          READ_FLAG( uiCode, "cbr_flag" );                          hrd->setCbrFlag( i, j, nalOrVcl, uiCode == 1 ? true : false  );
        }
      }
    }
  }
}

Void TDecCavlc::parseSPS(TComSPS* pcSPS)
{
#if ENC_DEC_TRACE
  xTraceSPSHeader ();
#endif

  UInt  uiCode;
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id");          pcSPS->setVPSId        ( uiCode );
  READ_CODE( 3,  uiCode, "sps_max_sub_layers_minus1" );          pcSPS->setMaxTLayers   ( uiCode+1 );
  assert(uiCode <= 6);

  READ_FLAG( uiCode, "sps_temporal_id_nesting_flag" );           pcSPS->setTemporalIdNestingFlag ( uiCode > 0 ? true : false );
  if ( pcSPS->getMaxTLayers() == 1 )
  {
    // sps_temporal_id_nesting_flag must be 1 when sps_max_sub_layers_minus1 is 0
    assert( uiCode == 1 );
  }

  parsePTL(pcSPS->getPTL(), 1, pcSPS->getMaxTLayers() - 1);
  READ_UVLC(     uiCode, "sps_seq_parameter_set_id" );           pcSPS->setSPSId( uiCode );
  assert(uiCode <= 15);

  READ_UVLC(     uiCode, "chroma_format_idc" );                  pcSPS->setChromaFormatIdc( ChromaFormat(uiCode) );
  assert(uiCode <= 3);

  if( pcSPS->getChromaFormatIdc() == CHROMA_444 )
  {
    READ_FLAG(     uiCode, "separate_colour_plane_flag");        assert(uiCode == 0);
  }

  READ_UVLC (    uiCode, "pic_width_in_luma_samples" );          pcSPS->setPicWidthInLumaSamples ( uiCode    );
  READ_UVLC (    uiCode, "pic_height_in_luma_samples" );         pcSPS->setPicHeightInLumaSamples( uiCode    );
  READ_FLAG(     uiCode, "conformance_window_flag");
  if (uiCode != 0)
  {
    Window &conf = pcSPS->getConformanceWindow();
    READ_UVLC(   uiCode, "conf_win_left_offset" );               conf.setWindowLeftOffset  ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC(   uiCode, "conf_win_right_offset" );              conf.setWindowRightOffset ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC(   uiCode, "conf_win_top_offset" );                conf.setWindowTopOffset   ( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC(   uiCode, "conf_win_bottom_offset" );             conf.setWindowBottomOffset( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
  }

  READ_UVLC(     uiCode, "bit_depth_luma_minus8" );
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setStreamBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);
  const UInt forceDecodeBitDepth = pcSPS->getForceDecodeBitDepth();
  if (forceDecodeBitDepth != 0)
  {
    uiCode = forceDecodeBitDepth - 8;
  }
#endif
  assert(uiCode <= 8);
  pcSPS->setBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);

#if O0043_BEST_EFFORT_DECODING
  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (Int) (6*(pcSPS->getStreamBitDepth(CHANNEL_TYPE_LUMA)-8)) );
#else
  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (Int) (6*uiCode) );
#endif

  READ_UVLC( uiCode,    "bit_depth_chroma_minus8" );
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setStreamBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
  if (forceDecodeBitDepth != 0)
  {
    uiCode = forceDecodeBitDepth - 8;
  }
#endif
  assert(uiCode <= 8);
  pcSPS->setBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (Int) (6*(pcSPS->getStreamBitDepth(CHANNEL_TYPE_CHROMA)-8)) );
#else
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (Int) (6*uiCode) );
#endif

  READ_UVLC( uiCode,    "log2_max_pic_order_cnt_lsb_minus4" );   pcSPS->setBitsForPOC( 4 + uiCode );
  assert(uiCode <= 12);

  UInt subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "sps_sub_layer_ordering_info_present_flag");

  for(UInt i=0; i <= pcSPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC ( uiCode, "sps_max_dec_pic_buffering_minus1[i]");
    pcSPS->setMaxDecPicBuffering( uiCode + 1, i);
    READ_UVLC ( uiCode, "sps_max_num_reorder_pics[i]" );
    pcSPS->setNumReorderPics(uiCode, i);
    READ_UVLC ( uiCode, "sps_max_latency_increase_plus1[i]");
    pcSPS->setMaxLatencyIncrease( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcSPS->getMaxTLayers()-1; i++)
      {
        pcSPS->setMaxDecPicBuffering(pcSPS->getMaxDecPicBuffering(0), i);
        pcSPS->setNumReorderPics(pcSPS->getNumReorderPics(0), i);
        pcSPS->setMaxLatencyIncrease(pcSPS->getMaxLatencyIncrease(0), i);
      }
      break;
    }
  }

  READ_UVLC( uiCode, "log2_min_luma_coding_block_size_minus3" );
  Int log2MinCUSize = uiCode + 3;
  pcSPS->setLog2MinCodingBlockSize(log2MinCUSize);
  READ_UVLC( uiCode, "log2_diff_max_min_luma_coding_block_size" );
  pcSPS->setLog2DiffMaxMinCodingBlockSize(uiCode);
  
  if (pcSPS->getPTL()->getGeneralPTL()->getLevelIdc() >= Level::LEVEL5)
  {
    assert(log2MinCUSize + pcSPS->getLog2DiffMaxMinCodingBlockSize() >= 5);
  }
  
  Int maxCUDepthDelta = uiCode;
  pcSPS->setMaxCUWidth  ( 1<<(log2MinCUSize + maxCUDepthDelta) );
  pcSPS->setMaxCUHeight ( 1<<(log2MinCUSize + maxCUDepthDelta) );
  READ_UVLC( uiCode, "log2_min_luma_transform_block_size_minus2" );   pcSPS->setQuadtreeTULog2MinSize( uiCode + 2 );

  READ_UVLC( uiCode, "log2_diff_max_min_luma_transform_block_size" ); pcSPS->setQuadtreeTULog2MaxSize( uiCode + pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxTrSize( 1<<(uiCode + pcSPS->getQuadtreeTULog2MinSize()) );

  READ_UVLC( uiCode, "max_transform_hierarchy_depth_inter" );    pcSPS->setQuadtreeTUMaxDepthInter( uiCode+1 );
  READ_UVLC( uiCode, "max_transform_hierarchy_depth_intra" );    pcSPS->setQuadtreeTUMaxDepthIntra( uiCode+1 );

  Int addCuDepth = max (0, log2MinCUSize - (Int)pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxTotalCUDepth( maxCUDepthDelta + addCuDepth  + getMaxCUDepthOffset(pcSPS->getChromaFormatIdc(), pcSPS->getQuadtreeTULog2MinSize()) );

  READ_FLAG( uiCode, "scaling_list_enabled_flag" );                 pcSPS->setScalingListFlag ( uiCode );
  if(pcSPS->getScalingListFlag())
  {
    READ_FLAG( uiCode, "sps_scaling_list_data_present_flag" );                 pcSPS->setScalingListPresentFlag ( uiCode );
    if(pcSPS->getScalingListPresentFlag ())
    {
      parseScalingList( &(pcSPS->getScalingList()) );
    }
  }
  READ_FLAG( uiCode, "amp_enabled_flag" );                          pcSPS->setUseAMP( uiCode );
  READ_FLAG( uiCode, "sample_adaptive_offset_enabled_flag" );       pcSPS->setUseSAO ( uiCode ? true : false );

  READ_FLAG( uiCode, "pcm_enabled_flag" ); pcSPS->setUsePCM( uiCode ? true : false );
  if( pcSPS->getUsePCM() )
  {
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_luma_minus1" );          pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_LUMA, 1 + uiCode );
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_chroma_minus1" );        pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_CHROMA, 1 + uiCode );
    READ_UVLC( uiCode, "log2_min_pcm_luma_coding_block_size_minus3" );   pcSPS->setPCMLog2MinSize (uiCode+3);
    READ_UVLC( uiCode, "log2_diff_max_min_pcm_luma_coding_block_size" ); pcSPS->setPCMLog2MaxSize ( uiCode+pcSPS->getPCMLog2MinSize() );
    READ_FLAG( uiCode, "pcm_loop_filter_disable_flag" );                 pcSPS->setPCMFilterDisableFlag ( uiCode ? true : false );
  }

  READ_UVLC( uiCode, "num_short_term_ref_pic_sets" );
  assert(uiCode <= 64);
  pcSPS->createRPSList(uiCode);

  TComRPSList* rpsList = pcSPS->getRPSList();
  TComReferencePictureSet* rps;

  for(UInt i=0; i< rpsList->getNumberOfReferencePictureSets(); i++)
  {
    rps = rpsList->getReferencePictureSet(i);
    parseShortTermRefPicSet(pcSPS,rps,i);
  }
  READ_FLAG( uiCode, "long_term_ref_pics_present_flag" );          pcSPS->setLongTermRefsPresent(uiCode);
  if (pcSPS->getLongTermRefsPresent())
  {
    READ_UVLC( uiCode, "num_long_term_ref_pics_sps" );
    pcSPS->setNumLongTermRefPicSPS(uiCode);
    for (UInt k = 0; k < pcSPS->getNumLongTermRefPicSPS(); k++)
    {
      READ_CODE( pcSPS->getBitsForPOC(), uiCode, "lt_ref_pic_poc_lsb_sps" );
      pcSPS->setLtRefPicPocLsbSps(k, uiCode);
      READ_FLAG( uiCode,  "used_by_curr_pic_lt_sps_flag[i]");
      pcSPS->setUsedByCurrPicLtSPSFlag(k, uiCode?1:0);
    }
  }
  READ_FLAG( uiCode, "sps_temporal_mvp_enable_flag" );            pcSPS->setTMVPFlagsPresent(uiCode);

  READ_FLAG( uiCode, "sps_strong_intra_smoothing_enable_flag" );  pcSPS->setUseStrongIntraSmoothing(uiCode);

  READ_FLAG( uiCode, "vui_parameters_present_flag" );             pcSPS->setVuiParametersPresentFlag(uiCode);

  if (pcSPS->getVuiParametersPresentFlag())
  {
    parseVUI(pcSPS->getVuiParameters(), pcSPS);
  }

  READ_FLAG( uiCode, "sps_extension_present_flag");
  if (uiCode)
  {
#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const char *syntaxStrings[]={ "sps_range_extension_flag",
                                         "sps_multilayer_extension_flag",
                                         "sps_extension_6bits[0]",
                                         "sps_extension_6bits[1]",
                                         "sps_extension_6bits[2]",
                                         "sps_extension_6bits[3]",
                                         "sps_extension_6bits[4]",
                                         "sps_extension_6bits[5]" };
#endif
    Bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS];

    for(Int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      sps_extension_flags[i] = uiCode!=0;
    }

    Bool bSkipTrailingExtensionBits=false;
    for(Int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
          case SPS_EXT__REXT:
            assert(!bSkipTrailingExtensionBits);
            {
              TComSPSRExt &spsRangeExtension = pcSPS->getSpsRangeExtension();
              READ_FLAG( uiCode, "transform_skip_rotation_enabled_flag");     spsRangeExtension.setTransformSkipRotationEnabledFlag(uiCode != 0);
              READ_FLAG( uiCode, "transform_skip_context_enabled_flag");      spsRangeExtension.setTransformSkipContextEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "implicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT, (uiCode != 0));
              READ_FLAG( uiCode, "explicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT, (uiCode != 0));
              READ_FLAG( uiCode, "extended_precision_processing_flag");       spsRangeExtension.setExtendedPrecisionProcessingFlag (uiCode != 0);
              READ_FLAG( uiCode, "intra_smoothing_disabled_flag");            spsRangeExtension.setIntraSmoothingDisabledFlag      (uiCode != 0);
              READ_FLAG( uiCode, "high_precision_offsets_enabled_flag");      spsRangeExtension.setHighPrecisionOffsetsEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "persistent_rice_adaptation_enabled_flag");  spsRangeExtension.setPersistentRiceAdaptationEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "cabac_bypass_alignment_enabled_flag");      spsRangeExtension.setCabacBypassAlignmentEnabledFlag  (uiCode != 0);
            }
            break;
          default:
            bSkipTrailingExtensionBits=true;
            break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "sps_extension_data_flag");
      }
    }
  }

  xReadRbspTrailingBits();
}

Void TDecCavlc::parseVPS(TComVPS* pcVPS)
{
#if ENC_DEC_TRACE
  xTraceVPSHeader ();
#endif
  UInt  uiCode;

  READ_CODE( 4,  uiCode,  "vps_video_parameter_set_id" );         pcVPS->setVPSId( uiCode );
  READ_FLAG( uiCode,      "vps_base_layer_internal_flag" );       assert(uiCode == 1);
  READ_FLAG( uiCode,      "vps_base_layer_available_flag" );      assert(uiCode == 1);
  READ_CODE( 6,  uiCode,  "vps_max_layers_minus1" );
  READ_CODE( 3,  uiCode,  "vps_max_sub_layers_minus1" );          pcVPS->setMaxTLayers( uiCode + 1 );    assert(uiCode+1 <= MAX_TLAYER);
  READ_FLAG(     uiCode,  "vps_temporal_id_nesting_flag" );       pcVPS->setTemporalNestingFlag( uiCode ? true:false );
  assert (pcVPS->getMaxTLayers()>1||pcVPS->getTemporalNestingFlag());
  READ_CODE( 16, uiCode,  "vps_reserved_0xffff_16bits" );         assert(uiCode == 0xffff);
  parsePTL ( pcVPS->getPTL(), true, pcVPS->getMaxTLayers()-1);
  UInt subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "vps_sub_layer_ordering_info_present_flag");
  for(UInt i = 0; i <= pcVPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC( uiCode,  "vps_max_dec_pic_buffering_minus1[i]" );    pcVPS->setMaxDecPicBuffering( uiCode + 1, i );
    READ_UVLC( uiCode,  "vps_max_num_reorder_pics[i]" );            pcVPS->setNumReorderPics( uiCode, i );
    READ_UVLC( uiCode,  "vps_max_latency_increase_plus1[i]" );      pcVPS->setMaxLatencyIncrease( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcVPS->getMaxTLayers()-1; i++)
      {
        pcVPS->setMaxDecPicBuffering(pcVPS->getMaxDecPicBuffering(0), i);
        pcVPS->setNumReorderPics(pcVPS->getNumReorderPics(0), i);
        pcVPS->setMaxLatencyIncrease(pcVPS->getMaxLatencyIncrease(0), i);
      }
      break;
    }
  }

  assert( pcVPS->getNumHrdParameters() < MAX_VPS_OP_SETS_PLUS1 );
  assert( pcVPS->getMaxNuhReservedZeroLayerId() < MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 );
  READ_CODE( 6, uiCode, "vps_max_layer_id" );                        pcVPS->setMaxNuhReservedZeroLayerId( uiCode );
  READ_UVLC(    uiCode, "vps_num_layer_sets_minus1" );               pcVPS->setMaxOpSets( uiCode + 1 );
  for( UInt opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx ++ )
  {
    // Operation point set
    for( UInt i = 0; i <= pcVPS->getMaxNuhReservedZeroLayerId(); i ++ )
    {
      READ_FLAG( uiCode, "layer_id_included_flag[opsIdx][i]" );   pcVPS->setLayerIdIncludedFlag( uiCode == 1 ? true : false, opsIdx, i );
    }
  }

  TimingInfo *timingInfo = pcVPS->getTimingInfo();
  READ_FLAG(       uiCode, "vps_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vps_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vps_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vps_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vps_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }

    READ_UVLC( uiCode, "vps_num_hrd_parameters" );                  pcVPS->setNumHrdParameters( uiCode );

    if( pcVPS->getNumHrdParameters() > 0 )
    {
      pcVPS->createHrdParamBuffer();
    }
    for( UInt i = 0; i < pcVPS->getNumHrdParameters(); i ++ )
    {
      READ_UVLC( uiCode, "hrd_layer_set_idx[i]" );                  pcVPS->setHrdOpSetIdx( uiCode, i );
      if( i > 0 )
      {
        READ_FLAG( uiCode, "cprms_present_flag[i]" );               pcVPS->setCprmsPresentFlag( uiCode == 1 ? true : false, i );
      }
      else
      {
        pcVPS->setCprmsPresentFlag( true, i );
      }

      parseHrdParameters(pcVPS->getHrdParameters(i), pcVPS->getCprmsPresentFlag( i ), pcVPS->getMaxTLayers() - 1);
    }
  }

  READ_FLAG( uiCode,  "vps_extension_flag" );
  if (uiCode)
  {
    while ( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "vps_extension_data_flag");
    }
  }

  xReadRbspTrailingBits();
}

Void TDecCavlc::parseSliceHeader (TComSlice* pcSlice, ParameterSetManager *parameterSetManager, const Int prevTid0POC)
{
  UInt  uiCode;
  Int   iCode;

#if ENC_DEC_TRACE
  xTraceSliceHeader();
#endif
  TComPPS* pps = NULL;
  TComSPS* sps = NULL;

  UInt firstSliceSegmentInPic;
  READ_FLAG( firstSliceSegmentInPic, "first_slice_segment_in_pic_flag" );
  if( pcSlice->getRapPicFlag())
  {
    READ_FLAG( uiCode, "no_output_of_prior_pics_flag" );  //ignored -- updated already
    pcSlice->setNoOutputPriorPicsFlag(uiCode ? true : false);
  }
  READ_UVLC (    uiCode, "slice_pic_parameter_set_id" );  pcSlice->setPPSId(uiCode);
  pps = parameterSetManager->getPPS(uiCode);
  //!KS: need to add error handling code here, if PPS is not available
  assert(pps!=0);
  sps = parameterSetManager->getSPS(pps->getSPSId());
  //!KS: need to add error handling code here, if SPS is not available
  assert(sps!=0);

  const ChromaFormat chFmt = sps->getChromaFormatIdc();
  const UInt numValidComp=getNumberValidComponents(chFmt);
  const Bool bChroma=(chFmt!=CHROMA_400);

  if( pps->getDependentSliceSegmentsEnabledFlag() && ( !firstSliceSegmentInPic ))
  {
    READ_FLAG( uiCode, "dependent_slice_segment_flag" );       pcSlice->setDependentSliceSegmentFlag(uiCode ? true : false);
  }
  else
  {
    pcSlice->setDependentSliceSegmentFlag(false);
  }
  Int numCTUs = ((sps->getPicWidthInLumaSamples()+sps->getMaxCUWidth()-1)/sps->getMaxCUWidth())*((sps->getPicHeightInLumaSamples()+sps->getMaxCUHeight()-1)/sps->getMaxCUHeight());
  UInt sliceSegmentAddress = 0;
  Int bitsSliceSegmentAddress = 0;
  while(numCTUs>(1<<bitsSliceSegmentAddress))
  {
    bitsSliceSegmentAddress++;
  }

  if(!firstSliceSegmentInPic)
  {
    READ_CODE( bitsSliceSegmentAddress, sliceSegmentAddress, "slice_segment_address" );
  }
  //set uiCode to equal slice start address (or dependent slice start address)
  pcSlice->setSliceSegmentCurStartCtuTsAddr( sliceSegmentAddress );// this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion table defined yet.
  pcSlice->setSliceSegmentCurEndCtuTsAddr(numCTUs);                // Set end as the last CTU of the picture.

  if (!pcSlice->getDependentSliceSegmentFlag())
  {
    pcSlice->setSliceCurStartCtuTsAddr(sliceSegmentAddress); // this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion table defined yet.
    pcSlice->setSliceCurEndCtuTsAddr(numCTUs);
  }

  if(!pcSlice->getDependentSliceSegmentFlag())
  {
    for (Int i = 0; i < pps->getNumExtraSliceHeaderBits(); i++)
    {
      READ_FLAG(uiCode, "slice_reserved_flag[]"); // ignored
    }

    READ_UVLC (    uiCode, "slice_type" );            pcSlice->setSliceType((SliceType)uiCode);
    if( pps->getOutputFlagPresentFlag() )
    {
      READ_FLAG( uiCode, "pic_output_flag" );    pcSlice->setPicOutputFlag( uiCode ? true : false );
    }
    else
    {
      pcSlice->setPicOutputFlag( true );
    }

    // if (separate_colour_plane_flag == 1)
    //   read colour_plane_id
    //   (separate_colour_plane_flag == 1) is not supported in this version of the standard.

    if( pcSlice->getIdrPicFlag() )
    {
      pcSlice->setPOC(0);
      TComReferencePictureSet* rps = pcSlice->getLocalRPS();
      rps->setNumberOfNegativePictures(0);
      rps->setNumberOfPositivePictures(0);
      rps->setNumberOfLongtermPictures(0);
      rps->setNumberOfPictures(0);
      pcSlice->setRPS(rps);
    }
    else
    {
      READ_CODE(sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb");
      Int iPOClsb = uiCode;
      Int iPrevPOC = prevTid0POC;
      Int iMaxPOClsb = 1<< sps->getBitsForPOC();
      Int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
      Int iPrevPOCmsb = iPrevPOC-iPrevPOClsb;
      Int iPOCmsb;
      if( ( iPOClsb  <  iPrevPOClsb ) && ( ( iPrevPOClsb - iPOClsb )  >=  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if( (iPOClsb  >  iPrevPOClsb )  && ( (iPOClsb - iPrevPOClsb )  >  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      }
      else
      {
        iPOCmsb = iPrevPOCmsb;
      }
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // For BLA picture types, POCmsb is set to 0.
        iPOCmsb = 0;
      }
      pcSlice->setPOC              (iPOCmsb+iPOClsb);

      TComReferencePictureSet* rps;
      rps = pcSlice->getLocalRPS();
      pcSlice->setRPS(rps);
      READ_FLAG( uiCode, "short_term_ref_pic_set_sps_flag" );
      if(uiCode == 0) // use short-term reference picture set explicitly signalled in slice header
      {
        parseShortTermRefPicSet(sps,rps, sps->getRPSList()->getNumberOfReferencePictureSets());
      }
      else // use reference to short-term reference picture set in PPS
      {
        Int numBits = 0;
        while ((1 << numBits) < sps->getRPSList()->getNumberOfReferencePictureSets())
        {
          numBits++;
        }
        if (numBits > 0)
        {
          READ_CODE( numBits, uiCode, "short_term_ref_pic_set_idx");
        }
        else
        {
          uiCode = 0;
       
        }
        *rps = *(sps->getRPSList()->getReferencePictureSet(uiCode));
      }
      if(sps->getLongTermRefsPresent())
      {
        Int offset = rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures();
        UInt numOfLtrp = 0;
        UInt numLtrpInSPS = 0;
        if (sps->getNumLongTermRefPicSPS() > 0)
        {
          READ_UVLC( uiCode, "num_long_term_sps");
          numLtrpInSPS = uiCode;
          numOfLtrp += numLtrpInSPS;
          rps->setNumberOfLongtermPictures(numOfLtrp);
        }
        Int bitsForLtrpInSPS = 0;
        while (sps->getNumLongTermRefPicSPS() > (1 << bitsForLtrpInSPS))
        {
          bitsForLtrpInSPS++;
        }
        READ_UVLC( uiCode, "num_long_term_pics");             rps->setNumberOfLongtermPictures(uiCode);
        numOfLtrp += uiCode;
        rps->setNumberOfLongtermPictures(numOfLtrp);
        Int maxPicOrderCntLSB = 1 << sps->getBitsForPOC();
        Int prevDeltaMSB = 0, deltaPocMSBCycleLT = 0;
        for(Int j=offset+rps->getNumberOfLongtermPictures()-1, k = 0; k < numOfLtrp; j--, k++)
        {
          Int pocLsbLt;
          if (k < numLtrpInSPS)
          {
            uiCode = 0;
            if (bitsForLtrpInSPS > 0)
            {
              READ_CODE(bitsForLtrpInSPS, uiCode, "lt_idx_sps[i]");
            }
            Bool usedByCurrFromSPS=sps->getUsedByCurrPicLtSPSFlag(uiCode);

            pocLsbLt = sps->getLtRefPicPocLsbSps(uiCode);
            rps->setUsed(j,usedByCurrFromSPS);
          }
          else
          {
            READ_CODE(sps->getBitsForPOC(), uiCode, "poc_lsb_lt"); pocLsbLt= uiCode;
            READ_FLAG( uiCode, "used_by_curr_pic_lt_flag");     rps->setUsed(j,uiCode);
          }
          READ_FLAG(uiCode,"delta_poc_msb_present_flag");
          Bool mSBPresentFlag = uiCode ? true : false;
          if(mSBPresentFlag)
          {
            READ_UVLC( uiCode, "delta_poc_msb_cycle_lt[i]" );
            Bool deltaFlag = false;
            //            First LTRP                               || First LTRP from SH
            if( (j == offset+rps->getNumberOfLongtermPictures()-1) || (j == offset+(numOfLtrp-numLtrpInSPS)-1) )
            {
              deltaFlag = true;
            }
            if(deltaFlag)
            {
              deltaPocMSBCycleLT = uiCode;
            }
            else
            {
              deltaPocMSBCycleLT = uiCode + prevDeltaMSB;
            }

            Int pocLTCurr = pcSlice->getPOC() - deltaPocMSBCycleLT * maxPicOrderCntLSB
                                        - iPOClsb + pocLsbLt;
            rps->setPOC     (j, pocLTCurr);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLTCurr);
            rps->setCheckLTMSBPresent(j,true);
          }
          else
          {
            rps->setPOC     (j, pocLsbLt);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLsbLt);
            rps->setCheckLTMSBPresent(j,false);

            // reset deltaPocMSBCycleLT for first LTRP from slice header if MSB not present
            if( j == offset+(numOfLtrp-numLtrpInSPS)-1 )
            {
              deltaPocMSBCycleLT = 0;
            }
          }
          prevDeltaMSB = deltaPocMSBCycleLT;
        }
        offset += rps->getNumberOfLongtermPictures();
        rps->setNumberOfPictures(offset);
      }
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // In the case of BLA picture types, rps data is read from slice header but ignored
        rps = pcSlice->getLocalRPS();
        rps->setNumberOfNegativePictures(0);
        rps->setNumberOfPositivePictures(0);
        rps->setNumberOfLongtermPictures(0);
        rps->setNumberOfPictures(0);
        pcSlice->setRPS(rps);
      }
      if (sps->getTMVPFlagsPresent())
      {
        READ_FLAG( uiCode, "slice_temporal_mvp_enabled_flag" );
        pcSlice->setEnableTMVPFlag( uiCode == 1 ? true : false );
      }
      else
      {
        pcSlice->setEnableTMVPFlag(false);
      }
    }
    if(sps->getUseSAO())
    {
      READ_FLAG(uiCode, "slice_sao_luma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, (Bool)uiCode);

      if (bChroma)
      {
        READ_FLAG(uiCode, "slice_sao_chroma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, (Bool)uiCode);
      }
    }

    if (pcSlice->getIdrPicFlag())
    {
      pcSlice->setEnableTMVPFlag(false);
    }
    if (!pcSlice->isIntra())
    {

      READ_FLAG( uiCode, "num_ref_idx_active_override_flag");
      if (uiCode)
      {
        READ_UVLC (uiCode, "num_ref_idx_l0_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_0, uiCode + 1 );
        if (pcSlice->isInterB())
        {
          READ_UVLC (uiCode, "num_ref_idx_l1_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_1, uiCode + 1 );
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
        }
      }
      else
      {
        pcSlice->setNumRefIdx(REF_PIC_LIST_0, pps->getNumRefIdxL0DefaultActive());
        if (pcSlice->isInterB())
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, pps->getNumRefIdxL1DefaultActive());
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1,0);
        }
      }
    }
    // }
    TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    if(!pcSlice->isIntra())
    {
      if( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 )
      {
        refPicListModification->setRefPicListModificationFlagL0( 0 );
      }
      else
      {
        READ_FLAG( uiCode, "ref_pic_list_modification_flag_l0" ); refPicListModification->setRefPicListModificationFlagL0( uiCode ? 1 : 0 );
      }

      if(refPicListModification->getRefPicListModificationFlagL0())
      {
        uiCode = 0;
        Int i = 0;
        Int numRpsCurrTempList0 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList0 > 1 )
        {
          Int length = 1;
          numRpsCurrTempList0 --;
          while ( numRpsCurrTempList0 >>= 1)
          {
            length ++;
          }
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
          {
            READ_CODE( length, uiCode, "list_entry_l0" );
            refPicListModification->setRefPicSetIdxL0(i, uiCode );
          }
        }
        else
        {
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
          {
            refPicListModification->setRefPicSetIdxL0(i, 0 );
          }
        }
      }
    }
    else
    {
      refPicListModification->setRefPicListModificationFlagL0(0);
    }
    if(pcSlice->isInterB())
    {
      if( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 )
      {
        refPicListModification->setRefPicListModificationFlagL1( 0 );
      }
      else
      {
        READ_FLAG( uiCode, "ref_pic_list_modification_flag_l1" ); refPicListModification->setRefPicListModificationFlagL1( uiCode ? 1 : 0 );
      }
      if(refPicListModification->getRefPicListModificationFlagL1())
      {
        uiCode = 0;
        Int i = 0;
        Int numRpsCurrTempList1 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList1 > 1 )
        {
          Int length = 1;
          numRpsCurrTempList1 --;
          while ( numRpsCurrTempList1 >>= 1)
          {
            length ++;
          }
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
          {
            READ_CODE( length, uiCode, "list_entry_l1" );
            refPicListModification->setRefPicSetIdxL1(i, uiCode );
          }
        }
        else
        {
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
          {
            refPicListModification->setRefPicSetIdxL1(i, 0 );
          }
        }
      }
    }
    else
    {
      refPicListModification->setRefPicListModificationFlagL1(0);
    }
    if (pcSlice->isInterB())
    {
      READ_FLAG( uiCode, "mvd_l1_zero_flag" );       pcSlice->setMvdL1ZeroFlag( (uiCode ? true : false) );
    }

    pcSlice->setCabacInitFlag( false ); // default
    if(pps->getCabacInitPresentFlag() && !pcSlice->isIntra())
    {
      READ_FLAG(uiCode, "cabac_init_flag");
      pcSlice->setCabacInitFlag( uiCode ? true : false );
    }

    if ( pcSlice->getEnableTMVPFlag() )
    {
      if ( pcSlice->getSliceType() == B_SLICE )
      {
        READ_FLAG( uiCode, "collocated_from_l0_flag" );
        pcSlice->setColFromL0Flag(uiCode);
      }
      else
      {
        pcSlice->setColFromL0Flag( 1 );
      }

      if ( pcSlice->getSliceType() != I_SLICE &&
          ((pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx(REF_PIC_LIST_0) > 1)||
           (pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx(REF_PIC_LIST_1) > 1)))
      {
        READ_UVLC( uiCode, "collocated_ref_idx" );
        pcSlice->setColRefIdx(uiCode);
      }
      else
      {
        pcSlice->setColRefIdx(0);
      }
    }
    if ( (pps->getUseWP() && pcSlice->getSliceType()==P_SLICE) || (pps->getWPBiPred() && pcSlice->getSliceType()==B_SLICE) )
    {
      xParsePredWeightTable(pcSlice, sps);
      pcSlice->initWpScaling(sps);
    }
    if (!pcSlice->isIntra())
    {
      READ_UVLC( uiCode, "five_minus_max_num_merge_cand");
      pcSlice->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - uiCode);
    }

    READ_SVLC( iCode, "slice_qp_delta" );
    pcSlice->setSliceQp (26 + pps->getPicInitQPMinus26() + iCode);

    assert( pcSlice->getSliceQp() >= -sps->getQpBDOffset(CHANNEL_TYPE_LUMA) );
    assert( pcSlice->getSliceQp() <=  51 );

    if (pps->getSliceChromaQpFlag())
    {
      if (numValidComp>COMPONENT_Cb)
      {
        READ_SVLC( iCode, "slice_cb_qp_offset" );
        pcSlice->setSliceChromaQpDelta(COMPONENT_Cb, iCode );
        assert( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb) >= -12 );
        assert( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb) <=  12 );
        assert( (pps->getQpOffset(COMPONENT_Cb) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cb)) >= -12 );
        assert( (pps->getQpOffset(COMPONENT_Cb) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cb)) <=  12 );
      }

      if (numValidComp>COMPONENT_Cr)
      {
        READ_SVLC( iCode, "slice_cr_qp_offset" );
        pcSlice->setSliceChromaQpDelta(COMPONENT_Cr, iCode );
        assert( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr) >= -12 );
        assert( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr) <=  12 );
        assert( (pps->getQpOffset(COMPONENT_Cr) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cr)) >= -12 );
        assert( (pps->getQpOffset(COMPONENT_Cr) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cr)) <=  12 );
      }
    }

    if (pps->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag())
    {
      READ_FLAG(uiCode, "cu_chroma_qp_offset_enabled_flag"); pcSlice->setUseChromaQpAdj(uiCode != 0);
    }
    else
    {
      pcSlice->setUseChromaQpAdj(false);
    }

    if (pps->getDeblockingFilterControlPresentFlag())
    {
      if(pps->getDeblockingFilterOverrideEnabledFlag())
      {
        READ_FLAG ( uiCode, "deblocking_filter_override_flag" );        pcSlice->setDeblockingFilterOverrideFlag(uiCode ? true : false);
      }
      else
      {
        pcSlice->setDeblockingFilterOverrideFlag(0);
      }
      if(pcSlice->getDeblockingFilterOverrideFlag())
      {
        READ_FLAG ( uiCode, "slice_disable_deblocking_filter_flag" );   pcSlice->setDeblockingFilterDisable(uiCode ? 1 : 0);
        if(!pcSlice->getDeblockingFilterDisable())
        {
          READ_SVLC( iCode, "slice_beta_offset_div2" );                       pcSlice->setDeblockingFilterBetaOffsetDiv2(iCode);
          assert(pcSlice->getDeblockingFilterBetaOffsetDiv2() >= -6 &&
                 pcSlice->getDeblockingFilterBetaOffsetDiv2() <=  6);
          READ_SVLC( iCode, "slice_tc_offset_div2" );                         pcSlice->setDeblockingFilterTcOffsetDiv2(iCode);
          assert(pcSlice->getDeblockingFilterTcOffsetDiv2() >= -6 &&
                 pcSlice->getDeblockingFilterTcOffsetDiv2() <=  6);
        }
      }
      else
      {
        pcSlice->setDeblockingFilterDisable   ( pps->getPicDisableDeblockingFilterFlag() );
        pcSlice->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( pps->getDeblockingFilterTcOffsetDiv2() );
      }
    }
    else
    {
      pcSlice->setDeblockingFilterDisable       ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterTcOffsetDiv2  ( 0 );
    }

    Bool isSAOEnabled = sps->getUseSAO() && (pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_LUMA) || (bChroma && pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA)));
    Bool isDBFEnabled = (!pcSlice->getDeblockingFilterDisable());

    if(pps->getLoopFilterAcrossSlicesEnabledFlag() && ( isSAOEnabled || isDBFEnabled ))
    {
      READ_FLAG( uiCode, "slice_loop_filter_across_slices_enabled_flag");
    }
    else
    {
      uiCode = pps->getLoopFilterAcrossSlicesEnabledFlag()?1:0;
    }
    pcSlice->setLFCrossSliceBoundaryFlag( (uiCode==1)?true:false);

  }

  std::vector<UInt> entryPointOffset;
  if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
  {
    UInt numEntryPointOffsets;
    UInt offsetLenMinus1;
    READ_UVLC(numEntryPointOffsets, "num_entry_point_offsets");
    if (numEntryPointOffsets>0)
    {
      READ_UVLC(offsetLenMinus1, "offset_len_minus1");
      entryPointOffset.resize(numEntryPointOffsets);
      for (UInt idx=0; idx<numEntryPointOffsets; idx++)
      {
        READ_CODE(offsetLenMinus1+1, uiCode, "entry_point_offset_minus1");
        entryPointOffset[ idx ] = uiCode + 1;
      }
    }
  }

  if(pps->getSliceHeaderExtensionPresentFlag())
  {
    READ_UVLC(uiCode,"slice_segment_header_extension_length");
    for(Int i=0; i<uiCode; i++)
    {
      UInt ignore;
      READ_CODE(8,ignore,"slice_segment_header_extension_data_byte");
    }
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,m_pcBitstream->readByteAlignment(),0);
#else
  m_pcBitstream->readByteAlignment();
#endif

  pcSlice->clearSubstreamSizes();

  if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
  {
    Int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

    // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
    for ( UInt curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
    {
      if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) < endOfSliceHeaderLocation )
      {
        endOfSliceHeaderLocation++;
      }
    }

    Int  curEntryPointOffset     = 0;
    Int  prevEntryPointOffset    = 0;
    for (UInt idx=0; idx<entryPointOffset.size(); idx++)
    {
      curEntryPointOffset += entryPointOffset[ idx ];

      Int emulationPreventionByteCount = 0;
      for ( UInt curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
      {
        if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) >= ( prevEntryPointOffset + endOfSliceHeaderLocation ) &&
             m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) <  ( curEntryPointOffset  + endOfSliceHeaderLocation ) )
        {
          emulationPreventionByteCount++;
        }
      }

      entryPointOffset[ idx ] -= emulationPreventionByteCount;
      prevEntryPointOffset = curEntryPointOffset;
      pcSlice->addSubstreamSize(entryPointOffset [ idx ] );
    }
  }

  return;
}

Void TDecCavlc::parsePTL( TComPTL *rpcPTL, Bool profilePresentFlag, Int maxNumSubLayersMinus1 )
{
  UInt uiCode;
  if(profilePresentFlag)
  {
    parseProfileTier(rpcPTL->getGeneralPTL(), false);
  }
  READ_CODE( 8, uiCode, "general_level_idc" );    rpcPTL->getGeneralPTL()->setLevelIdc(Level::Name(uiCode));

  for (Int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    READ_FLAG( uiCode, "sub_layer_profile_present_flag[i]" ); rpcPTL->setSubLayerProfilePresentFlag(i, uiCode);
    READ_FLAG( uiCode, "sub_layer_level_present_flag[i]"   ); rpcPTL->setSubLayerLevelPresentFlag  (i, uiCode);
  }

  if (maxNumSubLayersMinus1 > 0)
  {
    for (Int i = maxNumSubLayersMinus1; i < 8; i++)
    {
      READ_CODE(2, uiCode, "reserved_zero_2bits");
      assert(uiCode == 0);
    }
  }

  for(Int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    if( rpcPTL->getSubLayerProfilePresentFlag(i) )
    {
      parseProfileTier(rpcPTL->getSubLayerPTL(i), true);
    }
    if(rpcPTL->getSubLayerLevelPresentFlag(i))
    {
      READ_CODE( 8, uiCode, "sub_layer_level_idc[i]" );   rpcPTL->getSubLayerPTL(i)->setLevelIdc(Level::Name(uiCode));
    }
  }
}

#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecCavlc::parseProfileTier(ProfileTierLevel *ptl, const Bool bIsSubLayer)
#define PTL_TRACE_TEXT(txt) bIsSubLayer?("sub_layer_" txt) : ("general_" txt)
#else
Void TDecCavlc::parseProfileTier(ProfileTierLevel *ptl, const Bool /*bIsSubLayer*/)
#define PTL_TRACE_TEXT(txt) txt
#endif
{
  UInt uiCode;
  READ_CODE(2 , uiCode,   PTL_TRACE_TEXT("profile_space"                   )); ptl->setProfileSpace(uiCode);
  READ_FLAG(    uiCode,   PTL_TRACE_TEXT("tier_flag"                       )); ptl->setTierFlag    (uiCode ? Level::HIGH : Level::MAIN);
  READ_CODE(5 , uiCode,   PTL_TRACE_TEXT("profile_idc"                     )); ptl->setProfileIdc  (Profile::Name(uiCode));
  for(Int j = 0; j < 32; j++)
  {
    READ_FLAG(  uiCode,   PTL_TRACE_TEXT("profile_compatibility_flag[][j]" )); ptl->setProfileCompatibilityFlag(j, uiCode ? 1 : 0);
  }
  READ_FLAG(uiCode,       PTL_TRACE_TEXT("progressive_source_flag"         )); ptl->setProgressiveSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("interlaced_source_flag"          )); ptl->setInterlacedSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("non_packed_constraint_flag"      )); ptl->setNonPackedConstraintFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("frame_only_constraint_flag"      )); ptl->setFrameOnlyConstraintFlag(uiCode ? true : false);

  if (ptl->getProfileIdc() == Profile::MAINREXT           || ptl->getProfileCompatibilityFlag(Profile::MAINREXT) ||
      ptl->getProfileIdc() == Profile::HIGHTHROUGHPUTREXT || ptl->getProfileCompatibilityFlag(Profile::HIGHTHROUGHPUTREXT))
  {
    UInt maxBitDepth=16;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_12bit_constraint_flag"       )); if (uiCode) maxBitDepth=12;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_10bit_constraint_flag"       )); if (uiCode) maxBitDepth=10;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_8bit_constraint_flag"        )); if (uiCode) maxBitDepth=8;
    ptl->setBitDepthConstraint(maxBitDepth);
    ChromaFormat chromaFmtConstraint=CHROMA_444;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_422chroma_constraint_flag"   )); if (uiCode) chromaFmtConstraint=CHROMA_422;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_420chroma_constraint_flag"   )); if (uiCode) chromaFmtConstraint=CHROMA_420;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_monochrome_constraint_flag"  )); if (uiCode) chromaFmtConstraint=CHROMA_400;
    ptl->setChromaFormatConstraint(chromaFmtConstraint);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("intra_constraint_flag"           )); ptl->setIntraConstraintFlag(uiCode != 0);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("one_picture_only_constraint_flag")); ptl->setOnePictureOnlyConstraintFlag(uiCode != 0);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("lower_bit_rate_constraint_flag"  )); ptl->setLowerBitRateConstraintFlag(uiCode != 0);
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[0..15]"     ));
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[16..31]"    ));
    READ_CODE(2,  uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[32..33]"    ));
  }
  else
  {
    ptl->setBitDepthConstraint((ptl->getProfileIdc() == Profile::MAIN10)?10:8);
    ptl->setChromaFormatConstraint(CHROMA_420);
    ptl->setIntraConstraintFlag(false);
    ptl->setLowerBitRateConstraintFlag(true);
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[0..15]"     ));
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[16..31]"    ));
    READ_CODE(11, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[32..42]"    ));
  }

  if ((ptl->getProfileIdc() >= Profile::MAIN && ptl->getProfileIdc() <= Profile::HIGHTHROUGHPUTREXT) ||
       ptl->getProfileCompatibilityFlag(Profile::MAIN) ||
       ptl->getProfileCompatibilityFlag(Profile::MAIN10) ||
       ptl->getProfileCompatibilityFlag(Profile::MAINSTILLPICTURE) ||
       ptl->getProfileCompatibilityFlag(Profile::MAINREXT) ||
       ptl->getProfileCompatibilityFlag(Profile::HIGHTHROUGHPUTREXT) )
  {
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("inbld_flag"                      )); assert(uiCode == 0);
  }
  else
  {
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("reserved_zero_bit"               ));
  }
#undef PTL_TRACE_TEXT
}

Void TDecCavlc::parseTerminatingBit( UInt& ruiBit )
{
  ruiBit = false;
  Int iBitsLeft = m_pcBitstream->getNumBitsLeft();
  if(iBitsLeft <= 8)
  {
    UInt uiPeekValue = m_pcBitstream->peekBits(iBitsLeft);
    if (uiPeekValue == (1<<(iBitsLeft-1)))
    {
      ruiBit = true;
    }
  }
}

Void TDecCavlc::parseRemainingBytes( Bool noTrailingBytesExpected )
{
  if (noTrailingBytesExpected)
  {
    const UInt numberOfRemainingSubstreamBytes=m_pcBitstream->getNumBitsLeft();
    assert (numberOfRemainingSubstreamBytes == 0);
  }
  else
  {
    while (m_pcBitstream->getNumBitsLeft())
    {
      UInt trailingNullByte=m_pcBitstream->readByte();
      if (trailingNullByte!=0)
      {
        printf("Trailing byte should be 0, but has value %02x\n", trailingNullByte);
        assert(trailingNullByte==0);
      }
    }
  }
}

Void TDecCavlc::parseSkipFlag( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
{
  assert(0);
}

Void TDecCavlc::parseCUTransquantBypassFlag( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
{
  assert(0);
}

Void TDecCavlc::parseMVPIdx( Int& /*riMVPIdx*/ )
{
  assert(0);
}

Void TDecCavlc::parseSplitFlag     ( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
{
  assert(0);
}

Void TDecCavlc::parsePartSize( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
{
  assert(0);
}

Void TDecCavlc::parsePredMode( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
{
  assert(0);
}

/** Parse I_PCM information.
* \param pcCU pointer to CU
* \param uiAbsPartIdx CU index
* \param uiDepth CU depth
* \returns Void
*
* If I_PCM flag indicates that the CU is I_PCM, parse its PCM alignment bits and codes.
*/
Void TDecCavlc::parseIPCMInfo( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
{
  assert(0);
}

Void TDecCavlc::parseIntraDirLumaAng  ( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
{
  assert(0);
}

Void TDecCavlc::parseIntraDirChroma( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
{
  assert(0);
}

Void TDecCavlc::parseInterDir( TComDataCU* /*pcCU*/, UInt& /*ruiInterDir*/, UInt /*uiAbsPartIdx*/ )
{
  assert(0);
}

Void TDecCavlc::parseRefFrmIdx( TComDataCU* /*pcCU*/, Int& /*riRefFrmIdx*/, RefPicList /*eRefList*/ )
{
  assert(0);
}

Void TDecCavlc::parseMvd( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiPartIdx*/, UInt /*uiDepth*/, RefPicList /*eRefList*/ )
{
  assert(0);
}

Void TDecCavlc::parseCrossComponentPrediction( class TComTU& /*rTu*/, ComponentID /*compID*/ )
{
  assert(0);
}

Void TDecCavlc::parseDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  Int  iDQp;

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  READ_SVLC(iDQp, "delta_qp");
#else
  xReadSvlc( iDQp );
#endif

  Int qpBdOffsetY = pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
  const Int qp = (((Int) pcCU->getRefQP( uiAbsPartIdx ) + iDQp + 52 + 2*qpBdOffsetY )%(52+ qpBdOffsetY)) -  qpBdOffsetY;

  const UInt maxCUDepth        = pcCU->getSlice()->getSPS()->getMaxTotalCUDepth();
  const UInt maxCuDQPDepth     = pcCU->getSlice()->getPPS()->getMaxCuDQPDepth();
  const UInt doubleDepthDifference = ((maxCUDepth - maxCuDQPDepth)<<1);
  const UInt uiAbsQpCUPartIdx = (uiAbsPartIdx>>doubleDepthDifference)<<doubleDepthDifference ;
  const UInt uiQpCUDepth =   min(uiDepth,pcCU->getSlice()->getPPS()->getMaxCuDQPDepth()) ;

  pcCU->setQPSubParts( qp, uiAbsQpCUPartIdx, uiQpCUDepth );
}

Void TDecCavlc::parseChromaQpAdjustment( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
{
  assert(0);
}

Void TDecCavlc::parseCoeffNxN( TComTU &/*rTu*/, ComponentID /*compID*/ )
{
  assert(0);
}

Void TDecCavlc::parseTransformSubdivFlag( UInt& /*ruiSubdivFlag*/, UInt /*uiLog2TransformBlockSize*/ )
{
  assert(0);
}

Void TDecCavlc::parseQtCbf( TComTU &/*rTu*/, const ComponentID /*compID*/, const Bool /*lowestLevel*/ )
{
  assert(0);
}

Void TDecCavlc::parseQtRootCbf( UInt /*uiAbsPartIdx*/, UInt& /*uiQtRootCbf*/ )
{
  assert(0);
}

Void TDecCavlc::parseTransformSkipFlags (TComTU &/*rTu*/, ComponentID /*component*/)
{
  assert(0);
}

Void TDecCavlc::parseMergeFlag ( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/, UInt /*uiPUIdx*/ )
{
  assert(0);
}

Void TDecCavlc::parseMergeIndex ( TComDataCU* /*pcCU*/, UInt& /*ruiMergeIndex*/ )
{
  assert(0);
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! parse explicit wp tables
Void TDecCavlc::xParsePredWeightTable( TComSlice* pcSlice, const TComSPS *sps )
{
        WPScalingParam *wp;
  const ChromaFormat    chFmt        = sps->getChromaFormatIdc();
  const Int             numValidComp = Int(getNumberValidComponents(chFmt));
  const Bool            bChroma      = (chFmt!=CHROMA_400);
  const SliceType       eSliceType   = pcSlice->getSliceType();
  const Int             iNbRef       = (eSliceType == B_SLICE ) ? (2) : (1);
        UInt            uiLog2WeightDenomLuma=0, uiLog2WeightDenomChroma=0;
        UInt            uiTotalSignalledWeightFlags = 0;

  Int iDeltaDenom;
  // decode delta_luma_log2_weight_denom :
  READ_UVLC( uiLog2WeightDenomLuma, "luma_log2_weight_denom" );
  assert( uiLog2WeightDenomLuma <= 7 );
  if( bChroma )
  {
    READ_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
    assert((iDeltaDenom + (Int)uiLog2WeightDenomLuma)>=0);
    assert((iDeltaDenom + (Int)uiLog2WeightDenomLuma)<=7);
    uiLog2WeightDenomChroma = (UInt)(iDeltaDenom + uiLog2WeightDenomLuma);
  }

  for ( Int iNumRef=0 ; iNumRef<iNbRef ; iNumRef++ ) // loop over l0 and l1 syntax elements
  {
    RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( Int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);

      wp[COMPONENT_Y].uiLog2WeightDenom = uiLog2WeightDenomLuma;
      for(Int j=1; j<numValidComp; j++)
      {
        wp[j].uiLog2WeightDenom = uiLog2WeightDenomChroma;
      }

      UInt  uiCode;
      READ_FLAG( uiCode, iNumRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
      wp[COMPONENT_Y].bPresentFlag = ( uiCode == 1 );
      uiTotalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
    }
    if ( bChroma )
    {
      UInt  uiCode;
      for ( Int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        READ_FLAG( uiCode, iNumRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
        for(Int j=1; j<numValidComp; j++)
        {
          wp[j].bPresentFlag = ( uiCode == 1 );
        }
        uiTotalSignalledWeightFlags += 2*wp[COMPONENT_Cb].bPresentFlag;
      }
    }
    for ( Int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
      if ( wp[COMPONENT_Y].bPresentFlag )
      {
        Int iDeltaWeight;
        READ_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
        assert( iDeltaWeight >= -128 );
        assert( iDeltaWeight <=  127 );
        wp[COMPONENT_Y].iWeight = (iDeltaWeight + (1<<wp[COMPONENT_Y].uiLog2WeightDenom));
        READ_SVLC( wp[COMPONENT_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
        Int range=sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<sps->getBitDepth(CHANNEL_TYPE_LUMA))/2 : 128;
        assert( wp[0].iOffset >= -range );
        assert( wp[0].iOffset <   range );
      }
      else
      {
        wp[COMPONENT_Y].iWeight = (1 << wp[COMPONENT_Y].uiLog2WeightDenom);
        wp[COMPONENT_Y].iOffset = 0;
      }
      if ( bChroma )
      {
        if ( wp[COMPONENT_Cb].bPresentFlag )
        {
          Int range=sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<sps->getBitDepth(CHANNEL_TYPE_CHROMA))/2 : 128;
          for ( Int j=1 ; j<numValidComp ; j++ )
          {
            Int iDeltaWeight;
            READ_SVLC( iDeltaWeight, iNumRef==0?"delta_chroma_weight_l0[i]":"delta_chroma_weight_l1[i]" );
            assert( iDeltaWeight >= -128 );
            assert( iDeltaWeight <=  127 );
            wp[j].iWeight = (iDeltaWeight + (1<<wp[j].uiLog2WeightDenom));

            Int iDeltaChroma;
            READ_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
            assert( iDeltaChroma >= -4*range);
            assert( iDeltaChroma <   4*range);
            Int pred = ( range - ( ( range*wp[j].iWeight)>>(wp[j].uiLog2WeightDenom) ) );
            wp[j].iOffset = Clip3(-range, range-1, (iDeltaChroma + pred) );
          }
        }
        else
        {
          for ( Int j=1 ; j<numValidComp ; j++ )
          {
            wp[j].iWeight = (1 << wp[j].uiLog2WeightDenom);
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for ( Int iRefIdx=pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx<MAX_NUM_REF ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);

      wp[0].bPresentFlag = false;
      wp[1].bPresentFlag = false;
      wp[2].bPresentFlag = false;
    }
  }
  assert(uiTotalSignalledWeightFlags<=24);
}

/** decode quantization matrix
* \param scalingList quantization matrix information
*/
Void TDecCavlc::parseScalingList(TComScalingList* scalingList)
{
  UInt  code, sizeId, listId;
  Bool scalingListPredModeFlag;
  //for each size
  for(sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(listId = 0; listId <  SCALING_LIST_NUM; listId++)
    {
      if ((sizeId==SCALING_LIST_32x32) && (listId%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) != 0))
      {
        Int *src = scalingList->getScalingListAddress(sizeId, listId);
        const Int size = min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]);
        const Int *srcNextSmallerSize = scalingList->getScalingListAddress(sizeId-1, listId);
        for(Int i=0; i<size; i++)
        {
          src[i] = srcNextSmallerSize[i];
        }
        scalingList->setScalingListDC(sizeId,listId,(sizeId > SCALING_LIST_8x8) ? scalingList->getScalingListDC(sizeId-1, listId) : src[0]);
      }
      else
      {
        READ_FLAG( code, "scaling_list_pred_mode_flag");
        scalingListPredModeFlag = (code) ? true : false;
        scalingList->setScalingListPredModeFlag(sizeId, listId, scalingListPredModeFlag);
        if(!scalingListPredModeFlag) //Copy Mode
        {
          READ_UVLC( code, "scaling_list_pred_matrix_id_delta");

          if (sizeId==SCALING_LIST_32x32)
          {
            code*=(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES); // Adjust the decoded code for this size, to cope with the missing 32x32 chroma entries.
          }

          scalingList->setRefMatrixId (sizeId,listId,(UInt)((Int)(listId)-(code)));
          if( sizeId > SCALING_LIST_8x8 )
          {
            scalingList->setScalingListDC(sizeId,listId,((listId == scalingList->getRefMatrixId (sizeId,listId))? 16 :scalingList->getScalingListDC(sizeId, scalingList->getRefMatrixId (sizeId,listId))));
          }
          scalingList->processRefMatrix( sizeId, listId, scalingList->getRefMatrixId (sizeId,listId));

        }
        else //DPCM Mode
        {
          xDecodeScalingList(scalingList, sizeId, listId);
        }
      }
    }
  }

  return;
}
/** decode DPCM
* \param scalingList  quantization matrix information
* \param sizeId size index
* \param listId list index
*/
Void TDecCavlc::xDecodeScalingList(TComScalingList *scalingList, UInt sizeId, UInt listId)
{
  Int i,coefNum = min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]);
  Int data;
  Int scalingListDcCoefMinus8 = 0;
  Int nextCoef = SCALING_LIST_START_VALUE;
  UInt* scan  = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][sizeId==0 ? 2 : 3][sizeId==0 ? 2 : 3];
  Int *dst = scalingList->getScalingListAddress(sizeId, listId);

  if( sizeId > SCALING_LIST_8x8 )
  {
    READ_SVLC( scalingListDcCoefMinus8, "scaling_list_dc_coef_minus8");
    scalingList->setScalingListDC(sizeId,listId,scalingListDcCoefMinus8 + 8);
    nextCoef = scalingList->getScalingListDC(sizeId,listId);
  }

  for(i = 0; i < coefNum; i++)
  {
    READ_SVLC( data, "scaling_list_delta_coef");
    nextCoef = (nextCoef + data + 256 ) % 256;
    dst[scan[i]] = nextCoef;
  }
}

Bool TDecCavlc::xMoreRbspData()
{
  Int bitsLeft = m_pcBitstream->getNumBitsLeft();

  // if there are more than 8 bits, it cannot be rbsp_trailing_bits
  if (bitsLeft > 8)
  {
    return true;
  }

  UChar lastByte = m_pcBitstream->peekBits(bitsLeft);
  Int cnt = bitsLeft;

  // remove trailing bits equal to zero
  while ((cnt>0) && ((lastByte & 1) == 0))
  {
    lastByte >>= 1;
    cnt--;
  }
  // remove bit equal to one
  cnt--;

  // we should not have a negative number of bits
  assert (cnt>=0);

  // we have more data, if cnt is not zero
  return (cnt>0);
}

Void TDecCavlc::parseExplicitRdpcmMode( TComTU& /*rTu*/, ComponentID /*compID*/ )
{
  assert(0);
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

/** \file     TDecCAVLC.cpp
    \brief    CAVLC decoder class
*/

#include "TDecCAVLC.h"

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TDecCavlc::TDecCavlc()
{
  m_bAlfCtrl = false;
  m_uiMaxAlfCtrlDepth = 0;
}

TDecCavlc::~TDecCavlc()
{

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecCavlc::parsePPS(TComPPS* pcPPS)
{
#if HHI_NAL_UNIT_SYNTAX
  UInt  uiCode;

  xReadCode ( 2, uiCode ); //NalRefIdc
  xReadCode ( 1, uiCode ); assert( 0 == uiCode); // zero bit
  xReadCode ( 5, uiCode ); assert( NAL_UNIT_PPS == uiCode);//NalUnitType
#endif
  return;
}

Void TDecCavlc::parseSPS(TComSPS* pcSPS)
{
  UInt  uiCode;
#if HHI_NAL_UNIT_SYNTAX
  xReadCode ( 2, uiCode ); //NalRefIdc
  xReadCode ( 1, uiCode ); assert( 0 == uiCode); // zero bit
  xReadCode ( 5, uiCode ); assert( NAL_UNIT_SPS == uiCode);//NalUnitType
#endif
  // Structure
  xReadUvlc ( uiCode ); pcSPS->setWidth       ( uiCode    );
  xReadUvlc ( uiCode ); pcSPS->setHeight      ( uiCode    );
  xReadUvlc ( uiCode ); pcSPS->setPadX        ( uiCode    );
  xReadUvlc ( uiCode ); pcSPS->setPadY        ( uiCode    );

  xReadUvlc ( uiCode ); pcSPS->setMaxCUWidth  ( uiCode    ); g_uiMaxCUWidth  = uiCode;
  xReadUvlc ( uiCode ); pcSPS->setMaxCUHeight ( uiCode    ); g_uiMaxCUHeight = uiCode;
#if HHI_RQT
  UInt uiMaxCUDepthCorrect = 0;
  xReadUvlc ( uiMaxCUDepthCorrect );
#else
  xReadUvlc ( uiCode ); pcSPS->setMaxCUDepth  ( uiCode+1  ); g_uiMaxCUDepth  = uiCode + 1;
#endif
  
  // Transform
  xReadUvlc ( uiCode ); pcSPS->setMinTrDepth  ( uiCode    );
  xReadUvlc ( uiCode ); pcSPS->setMaxTrDepth  ( uiCode    );

#if HHI_RQT
  xReadFlag( uiCode ); pcSPS->setQuadtreeTUFlag( uiCode != 0 );
  if( pcSPS->getQuadtreeTUFlag() )
  {
    xReadUvlc( uiCode ); pcSPS->setQuadtreeTULog2MinSize( uiCode + 2 );
    if( pcSPS->getQuadtreeTULog2MinSize() < 6 )
    {
      xReadUvlc( uiCode ); pcSPS->setQuadtreeTULog2MaxSize( uiCode + pcSPS->getQuadtreeTULog2MinSize() );
    }
#if HHI_RQT_DEPTH
    xReadUvlc ( uiCode ); pcSPS->setQuadtreeTUMaxDepth( uiCode+1 );
#endif	
  }
  
  if( pcSPS->getQuadtreeTUFlag() )
  {
    g_uiAddCUDepth = 0;
    while( ( pcSPS->getMaxCUWidth() >> uiMaxCUDepthCorrect ) > ( 1 << ( pcSPS->getQuadtreeTULog2MinSize() + g_uiAddCUDepth )  ) ) g_uiAddCUDepth++;
    
    pcSPS->setMaxCUDepth( uiMaxCUDepthCorrect+g_uiAddCUDepth  ); g_uiMaxCUDepth  = uiMaxCUDepthCorrect+g_uiAddCUDepth;
  }
  else
  {
    pcSPS->setMaxCUDepth( uiMaxCUDepthCorrect+1  ); g_uiMaxCUDepth  = uiMaxCUDepthCorrect + 1;
  }  
#endif  // Max transform size
  xReadUvlc ( uiCode ); pcSPS->setMaxTrSize   ( (uiCode == 0) ? 2 : (1<<(uiCode+1)) );

  // Tool on/off
  xReadFlag( uiCode ); pcSPS->setUseALF ( uiCode ? true : false );
  xReadFlag( uiCode ); pcSPS->setUseDQP ( uiCode ? true : false );
  xReadFlag( uiCode ); pcSPS->setUseWPG ( uiCode ? true : false );
  xReadFlag( uiCode ); pcSPS->setUseLDC ( uiCode ? true : false );
  xReadFlag( uiCode ); pcSPS->setUseQBO ( uiCode ? true : false );
#ifdef QC_AMVRES
    xReadFlag( uiCode ); pcSPS->setUseAMVRes ( uiCode ? true : false );
#endif
#if HHI_ALLOW_CIP_SWITCH
  xReadFlag( uiCode ); pcSPS->setUseCIP ( uiCode ? true : false ); // BB:
#endif
  xReadFlag( uiCode ); pcSPS->setUseROT ( uiCode ? true : false ); // BB:
#if HHI_AIS
  xReadFlag( uiCode ); pcSPS->setUseAIS ( uiCode ? true : false ); // BB:
#endif
#if HHI_MRG
  xReadFlag( uiCode ); pcSPS->setUseMRG ( uiCode ? true : false ); // SOPH:
#endif
#if HHI_IMVP
  xReadFlag( uiCode ); pcSPS->setUseIMP ( uiCode ? true : false ); // SOPH:
#endif
#ifdef DCM_PBIC
  xReadFlag( uiCode ); pcSPS->setUseIC  ( uiCode ? true : false );
#endif

  xReadFlag( uiCode ); pcSPS->setUseAMP ( uiCode ? true : false );
#if HHI_RMP_SWITCH
  xReadFlag( uiCode ); pcSPS->setUseRMP( uiCode ? true : false );
#endif
  // number of taps for DIF
  xReadUvlc( uiCode ); pcSPS->setDIFTap ( (uiCode+2)<<1 );  // 4, 6, 8, 10, 12
#if SAMSUNG_CHROMA_IF_EXT
  xReadUvlc( uiCode ); pcSPS->setDIFTapC ( (uiCode+1)<<1 );  //2, 4, 6, 8, 10, 12
#endif

  // AMVP mode for each depth (AM_NONE or AM_EXPL)
  for (Int i = 0; i < pcSPS->getMaxCUDepth(); i++)
  {
    xReadFlag( uiCode );
    pcSPS->setAMVPMode( i, (AMVP_MODE)uiCode );
  }

  // Bit-depth information
  xReadUvlc( uiCode ); pcSPS->setBitDepth     ( uiCode+8 ); g_uiBitDepth     = uiCode + 8;
  xReadUvlc( uiCode ); pcSPS->setBitIncrement ( uiCode   ); g_uiBitIncrement = uiCode;

  xReadCode( 8, uiCode ); pcSPS->setBalancedCPUs( uiCode );

  g_uiBASE_MAX  = ((1<<(g_uiBitDepth))-1);

#if IBDI_NOCLIP_RANGE
  g_uiIBDI_MAX  = g_uiBASE_MAX << g_uiBitIncrement;
#else
  g_uiIBDI_MAX  = ((1<<(g_uiBitDepth+g_uiBitIncrement))-1);
#endif

#if HHI_RQT
  if( !pcSPS->getQuadtreeTUFlag() )
#endif    
  {
    g_uiAddCUDepth = 0;
    if( ((g_uiMaxCUWidth>>(g_uiMaxCUDepth-1)) > pcSPS->getMaxTrSize()) )
    {
      while( (g_uiMaxCUWidth>>(g_uiMaxCUDepth-1)) > (pcSPS->getMaxTrSize()<<g_uiAddCUDepth) ) g_uiAddCUDepth++;
    }
    g_uiMaxCUDepth += g_uiAddCUDepth;
    g_uiAddCUDepth++;
  }
  return;
}

Void TDecCavlc::parseSliceHeader (TComSlice*& rpcSlice)
{
  UInt  uiCode;
  Int   iCode;
#if HHI_NAL_UNIT_SYNTAX
  xReadCode ( 2, uiCode ); //NalRefIdc
  xReadCode ( 1, uiCode ); assert( 0 == uiCode); // zero bit
  xReadCode ( 5, uiCode ); assert( NAL_UNIT_CODED_SLICE == uiCode);//NalUnitType
#endif
  xReadCode (10, uiCode);  rpcSlice->setPOC              (uiCode);             // 9 == SPS->Log2MaxFrameNum()
  xReadUvlc (   uiCode);  rpcSlice->setSliceType        ((SliceType)uiCode);
  xReadSvlc (    iCode);  rpcSlice->setSliceQp          (iCode);

  xReadFlag ( uiCode );
  if( uiCode )
  {
    xReadFlag ( uiCode );
    uiCode++;
    rpcSlice->setSymbolMode( uiCode );
    xReadFlag( uiCode );
    rpcSlice->setMultiCodeword( uiCode == 1 );
    if( rpcSlice->getSymbolMode() == 2 && ! rpcSlice->getMultiCodeword() )
    {
      xReadUvlc( uiCode );
      rpcSlice->setMaxPIPEDelay( uiCode << 6 );
    }
  }
  else
  {
    xReadFlag ( uiCode );
    rpcSlice->setSymbolMode( uiCode ? 3 : 0 );
  }

  if (!rpcSlice->isIntra())
    xReadFlag (   uiCode);
  else
    uiCode = 1;

  rpcSlice->setReferenced       (uiCode ? true : false);

#ifdef ROUNDING_CONTROL_BIPRED
  if(!rpcSlice->isIntra())
  {
	xReadFlag( uiCode );
	Bool b = (uiCode != 0);
	rpcSlice->setRounding(b);
  }
#endif

  xReadFlag (   uiCode);  rpcSlice->setLoopFilterDisable(uiCode ? 1 : 0);

  if (!rpcSlice->isIntra())
  {
    xReadCode (3, uiCode);  rpcSlice->setNumRefIdx      (REF_PIC_LIST_0, uiCode);
  }
  else
  {
    rpcSlice->setNumRefIdx(REF_PIC_LIST_0, 0);
  }
  if (rpcSlice->isInterB())
  {
    xReadCode (3, uiCode);  rpcSlice->setNumRefIdx      (REF_PIC_LIST_1, uiCode);
  }
  else
  {
    rpcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
  }

  xReadFlag (uiCode);     rpcSlice->setDRBFlag          (uiCode ? 1 : 0);
  if ( !rpcSlice->getDRBFlag() )
  {
    xReadCode(2, uiCode); rpcSlice->setERBIndex( (ERBIndex)uiCode );    assert (uiCode == ERB_NONE || uiCode == ERB_LTR);
  }

  if (!rpcSlice->isIntra())
  {
    Int  iNumPredDir = rpcSlice->isInterP() ? 1 : 2;

    if (rpcSlice->getSPS()->getUseWPG())
    {
      for (Int n=0; n<iNumPredDir; n++)
      {
        RefPicList eRefPicList = (RefPicList)n;

        xReadCode (1, uiCode);
        rpcSlice->setWPmode(eRefPicList, uiCode);

        if (rpcSlice->getWPmode(eRefPicList))
        {
          rpcSlice->addEffectMode(eRefPicList, EFF_WP_SO);
#if !GRF_WP_CHROMA
          rpcSlice->initWPParam(eRefPicList, EFF_WP_SO, 1);
          rpcSlice->initWPParam(eRefPicList, EFF_WP_SO, 2);
#endif
          UInt uiTemp;
          Int iWeight, iOffset;

          xReadUvlc(uiTemp);
          iWeight = ( uiTemp & 1) ? (Int)((uiTemp+1)>>1) : -(Int)(uiTemp>>1);
          iWeight=iWeight+32;
          rpcSlice->setWPWeight(eRefPicList, EFF_WP_SO,0,iWeight);

          xReadUvlc(uiTemp);
          iOffset = ( uiTemp & 1) ? (Int)((uiTemp+1)>>1) : -(Int)(uiTemp>>1);
          rpcSlice->setWPOffset(eRefPicList, EFF_WP_SO,0,iOffset);

#if GRF_WP_CHROMA
          xReadUvlc(uiTemp);
          iWeight = ( uiTemp & 1) ? (Int)((uiTemp+1)>>1) : -(Int)(uiTemp>>1);
          iWeight=iWeight+32;
          rpcSlice->setWPWeight(eRefPicList, EFF_WP_SO,1,iWeight);

          xReadUvlc(uiTemp);
          iOffset = ( uiTemp & 1) ? (Int)((uiTemp+1)>>1) : -(Int)(uiTemp>>1);
          rpcSlice->setWPOffset(eRefPicList, EFF_WP_SO,1,iOffset);

          xReadUvlc(uiTemp);
          iWeight = ( uiTemp & 1) ? (Int)((uiTemp+1)>>1) : -(Int)(uiTemp>>1);
          iWeight=iWeight+32;
          rpcSlice->setWPWeight(eRefPicList, EFF_WP_SO,2,iWeight);

          xReadUvlc(uiTemp);
          iOffset = ( uiTemp & 1) ? (Int)((uiTemp+1)>>1) : -(Int)(uiTemp>>1);
          rpcSlice->setWPOffset(eRefPicList, EFF_WP_SO,2,iOffset);
#endif
        }
      }
    }
  }

#if HHI_INTERP_FILTER
  xReadUvlc( uiCode ); rpcSlice->setInterpFilterType( uiCode );
#endif

#if AMVP_NEIGH_COL
  if ( rpcSlice->getSliceType() == B_SLICE )
  {
    xReadFlag (uiCode);
    rpcSlice->setColDir(uiCode);
  }
#endif
#ifdef EDGE_BASED_PREDICTION
  xReadFlag(uiCode);
  rpcSlice->setEdgePredictionEnable(uiCode);
  if( rpcSlice->getEdgePredictionEnable() )
  {
    xReadCode(8, uiCode);
    rpcSlice->setEdgeDetectionThreshold(((Int)uiCode<<8));
  }
#endif //EDGE_BASED_PREDICTION
  return;
}

Void TDecCavlc::resetEntropy          (TComSlice* pcSlice)
{
  m_bRunLengthCoding = ! pcSlice->isIntra();
  m_uiRun = 0;

  ::memcpy(m_uiLPTableD8,        g_auiLPTableD8,        10*128*sizeof(UInt));
  ::memcpy(m_uiLPTableD4,        g_auiLPTableD4,        3*32*sizeof(UInt));
  ::memcpy(m_uiLastPosVlcIndex, g_auiLastPosVlcIndex, 10*sizeof(UInt));

#if LCEC_PHASE2
  ::memcpy(m_uiCBPTableD,        g_auiCBPTableD,        2*8*sizeof(UInt));
  m_uiCbpVlcIdx[0] = 0;
  m_uiCbpVlcIdx[1] = 0;
#endif

#if LCEC_PHASE2
  ::memcpy(m_uiMI1TableD,        g_auiMI1TableD,        8*sizeof(UInt));
  ::memcpy(m_uiMI2TableD,        g_auiMI2TableD,        15*sizeof(UInt));

  m_uiMITableVlcIdx = 0;

#endif


}

Void TDecCavlc::parseTerminatingBit( UInt& ruiBit )
{
  xReadFlag( ruiBit );
}

Void TDecCavlc::parseCIPflag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  xReadFlag( uiSymbol );
  pcCU->setCIPflagSubParts( (UChar)uiSymbol, uiAbsPartIdx, uiDepth );
}

Void TDecCavlc::parseROTindex  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt  uiSymbol;
  UInt  indexROT = 0;
  Int    dictSize = ROT_DICT;

  switch (dictSize)
  {
  case 9:
    {
      xReadFlag( uiSymbol );
      if ( !uiSymbol )
      {
        xReadFlag( uiSymbol );
        indexROT  = uiSymbol;
        xReadFlag( uiSymbol );
        indexROT |= uiSymbol << 1;
        xReadFlag( uiSymbol );
        indexROT |= uiSymbol << 2;
        indexROT++;
      }
    }
    break;
  case 4:
    {
      xReadFlag( uiSymbol );
      indexROT  = uiSymbol;
      xReadFlag( uiSymbol );
      indexROT |= uiSymbol << 1;
    }
    break;
  case 2:
    {
      xReadFlag( uiSymbol );
      if ( !uiSymbol ) indexROT =1;
    }
    break;
  case 5:
    {
      xReadFlag( uiSymbol );
      if ( !uiSymbol )
      {
        xReadFlag( uiSymbol );
        indexROT  = uiSymbol;
        xReadFlag( uiSymbol );
        indexROT |= uiSymbol << 1;
        indexROT++;
      }
    }
    break;
  case 1:
    {
    }
    break;
  }
  pcCU->setROTindexSubParts( indexROT, uiAbsPartIdx, uiDepth );

  return;
}

Void TDecCavlc::parseAlfCtrlDepth              ( UInt& ruiAlfCtrlDepth )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, g_uiMaxCUDepth-1);
  ruiAlfCtrlDepth = uiSymbol;
}

Void TDecCavlc::parseAlfCtrlFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if (!m_bAlfCtrl)
    return;

  if( uiDepth > m_uiMaxAlfCtrlDepth && !pcCU->isFirstAbsZorderIdxInDepth(uiAbsPartIdx, m_uiMaxAlfCtrlDepth))
  {
    return;
  }

  UInt uiSymbol;
  xReadFlag( uiSymbol );

  if (uiDepth > m_uiMaxAlfCtrlDepth)
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, m_uiMaxAlfCtrlDepth);
  }
  else
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, uiDepth );
  }
}

Void TDecCavlc::parseSkipFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
#if HHI_MRG && !SAMSUNG_MRG_SKIP_DIRECT
  if ( pcCU->getSlice()->getSPS()->getUseMRG() )
  {
    return;
  }
#endif

  if( pcCU->getSlice()->isIntra() )
  {
    return;
  }

  UInt uiSymbol;
  xReadFlag( uiSymbol );

  if( uiSymbol )
  {
    pcCU->setPredModeSubParts( MODE_SKIP,  uiAbsPartIdx, uiDepth );
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );

    TComMv cZeroMv(0,0);
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd    ( cZeroMv, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd    ( cZeroMv, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );

#ifdef DCM_PBIC
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      TComIc cDefaultIc;
      pcCU->getCUIcField()->setAllIcd ( cDefaultIc, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
    }
#endif

    pcCU->setTrIdxSubParts( 0, uiAbsPartIdx, uiDepth );
    pcCU->setCbfSubParts  ( 0, 0, 0, uiAbsPartIdx, uiDepth );

    if ( pcCU->getSlice()->isInterP() )
    {
      pcCU->setInterDirSubParts( 1, uiAbsPartIdx, 0, uiDepth );

      if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 )
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx(  0, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
      if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0 )
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( NOT_VALID, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
    }
    else
    {
      pcCU->setInterDirSubParts( 3, uiAbsPartIdx, 0, uiDepth );

      if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) > 0 )
        pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllRefIdx(  0, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
      if ( pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) > 0 )
        pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllRefIdx( 0, SIZE_2Nx2N, uiAbsPartIdx, 0, uiDepth );
    }
  }
}

Void TDecCavlc::parseMVPIdx      ( TComDataCU* pcCU, Int& riMVPIdx, Int iMVPNum, UInt uiAbsPartIdx, UInt uiDepth, RefPicList eRefList )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, iMVPNum-1);
  riMVPIdx = uiSymbol;
}

#ifdef DCM_PBIC
Void TDecCavlc::parseICPIdx      ( TComDataCU* pcCU, Int& riICPIdx, Int iICPNum, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  xReadUnaryMaxSymbol(uiSymbol, iICPNum-1);
  riICPIdx = uiSymbol;
}
#endif

Void TDecCavlc::parseSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
  {
    pcCU->setDepthSubParts( uiDepth, uiAbsPartIdx );
    return ;
  }

  UInt uiSymbol;
  xReadFlag( uiSymbol );
  pcCU->setDepthSubParts( uiDepth + uiSymbol, uiAbsPartIdx );

  return ;
}

Void TDecCavlc::parsePartSize( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
#if HHI_MRG && !HHI_MRG_PU
  if ( pcCU->getMergeFlag( uiAbsPartIdx ) )
  {
    return;
  }
#endif

  if ( pcCU->isSkip( uiAbsPartIdx ) )
  {
    pcCU->setPartSizeSubParts( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
    return ;
  }

  UInt uiSymbol, uiMode = 0;
  PartSize eMode;

  if ( pcCU->isIntra( uiAbsPartIdx ) )
  {
    xReadFlag( uiSymbol );
    eMode = uiSymbol ? SIZE_2Nx2N : SIZE_NxN;
  }
  else
  {
#if HHI_RMP_SWITCH
    if ( !pcCU->getSlice()->getSPS()->getUseRMP() && !pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
    {
      xReadFlag( uiSymbol );
      if( uiSymbol )
        uiMode = 0;
      else
        uiMode = 3;
    }
    else
#endif
    {
      UInt uiMaxNumBits = 3;
      for ( UInt ui = 0; ui < uiMaxNumBits; ui++ )
      {
        xReadFlag( uiSymbol );
        if ( uiSymbol )
        {
          break;
        }
        uiMode++;
      }
    }
    eMode = (PartSize) uiMode;

    if (pcCU->getSlice()->isInterB() && uiMode == 3)
    {
#if HHI_DISABLE_INTER_NxN_SPLIT
       uiSymbol = 0;
       if( g_uiMaxCUWidth>>uiDepth == 8 )
#endif
      xReadFlag( uiSymbol );
      if (uiSymbol == 0)
      {
        pcCU->setPredModeSubParts( MODE_INTRA, uiAbsPartIdx, uiDepth );
        xReadFlag( uiSymbol );
        if (uiSymbol == 0)
          eMode = SIZE_2Nx2N;
      }
    }

#if HHI_RMP_SWITCH
    if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) && pcCU->getSlice()->getSPS()->getUseRMP() )
#else
    if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) )
#endif
    {
      if (eMode == SIZE_2NxN)
      {
        xReadFlag(uiSymbol);
        if (uiSymbol == 0)
        {
          xReadFlag(uiSymbol);
          eMode = (uiSymbol == 0? SIZE_2NxnU : SIZE_2NxnD);
        }
      }
      else if (eMode == SIZE_Nx2N)
      {
        xReadFlag(uiSymbol);
        if (uiSymbol == 0)
        {
          xReadFlag(uiSymbol);
          eMode = (uiSymbol == 0? SIZE_nLx2N : SIZE_nRx2N);
        }
      }
    }
#if HHI_RMP_SWITCH
    if ( pcCU->getSlice()->getSPS()->getAMPAcc( uiDepth ) && !pcCU->getSlice()->getSPS()->getUseRMP() )
    {
      if ( eMode == SIZE_2NxN )
      {
        xReadFlag(uiSymbol);
        eMode = (uiSymbol == 0? SIZE_2NxnU : SIZE_2NxnD);
      }
      else if ( eMode == SIZE_Nx2N )
      {
        xReadFlag(uiSymbol);
        eMode = (uiSymbol == 0? SIZE_nLx2N : SIZE_nRx2N);
      }
    }
#endif
  }

  pcCU->setPartSizeSubParts( eMode, uiAbsPartIdx, uiDepth );
  pcCU->setSizeSubParts( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );

  UInt uiTrLevel = 0;

  UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(uiAbsPartIdx)]+2;
  UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
  uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;

  if( pcCU->getPredictionMode(uiAbsPartIdx) == MODE_INTRA )
  {
    if( pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_NxN )
    {
      pcCU->setTrIdxSubParts( 1+uiTrLevel, uiAbsPartIdx, uiDepth );
    }
    else
    {
      pcCU->setTrIdxSubParts( uiTrLevel, uiAbsPartIdx, uiDepth );
    }
  }
}

Void TDecCavlc::parsePredMode( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
#if HHI_MRG && !HHI_MRG_PU
  if ( pcCU->getMergeFlag( uiAbsPartIdx ) )
  {
    pcCU->setPredModeSubParts( MODE_INTER, uiAbsPartIdx, uiDepth );
    return;
  }
#endif

  if( pcCU->getSlice()->isIntra() )
  {
    pcCU->setPredModeSubParts( MODE_INTRA, uiAbsPartIdx, uiDepth );
    return ;
  }

  UInt uiSymbol;
  Int  iPredMode = MODE_INTER;

#if HHI_MRG && !SAMSUNG_MRG_SKIP_DIRECT
  if ( !pcCU->getSlice()->getSPS()->getUseMRG() )
  {
    xReadFlag( uiSymbol );
    if ( uiSymbol == 0 )
    {
      iPredMode = MODE_SKIP;
    }
  }
#else
    xReadFlag( uiSymbol );
    if ( uiSymbol == 0 )
    {
      iPredMode = MODE_SKIP;
    }
#endif

  if ( pcCU->getSlice()->isInterB() )
  {
    pcCU->setPredModeSubParts( (PredMode)iPredMode, uiAbsPartIdx, uiDepth );
    return;
  }

  if ( iPredMode != MODE_SKIP )
  {
    xReadFlag( uiSymbol );
    iPredMode += uiSymbol;
  }

  pcCU->setPredModeSubParts( (PredMode)iPredMode, uiAbsPartIdx, uiDepth );
}

#if PLANAR_INTRA
// Temporary VLC function
UInt TDecCavlc::xParsePlanarVlc( )
{
  Bool bDone    = false;
  UInt uiZeroes = 0;
  UInt uiBit;
  UInt uiCodeword;

  while (!bDone)
  {
    xReadFlag( uiBit );

    if ( uiBit )
    {
      xReadFlag( uiCodeword );
      bDone = true;
    }
    else
      uiZeroes++;
  }

  return ( ( uiZeroes << 1 ) + uiCodeword );
}

Int TDecCavlc::xParsePlanarDelta( TextType ttText )
{
  /* Planar quantization
  Y        qY              cW
  0-3   :  0,1,2,3         0-3
  4-15  :  4,6,8..14       4-9
  16-63 : 18,22,26..62    10-21
  64-.. : 68,76...        22-
  */
  UInt bDeltaNegative = 0;
  Int  iDelta         = xParsePlanarVlc();

  if( iDelta > 21 )
    iDelta = ( ( iDelta - 14 ) << 3 ) + 4;
  else if( iDelta > 9 )
    iDelta = ( ( iDelta - 6 ) << 2 ) + 2;
  else if( iDelta > 3 )
    iDelta = ( iDelta - 2 ) << 1;

  if( iDelta > 0 )
  {
    xReadFlag( bDeltaNegative );

    if( bDeltaNegative )
      iDelta = -iDelta;
  }

  return iDelta;
}

Void TDecCavlc::parsePlanarInfo( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  xReadFlag( uiSymbol );

  if ( uiSymbol )
  {
    Int iPlanarFlag   = 1;
    Int iPlanarDeltaY = xParsePlanarDelta( TEXT_LUMA );
    Int iPlanarDeltaU = 0;
    Int iPlanarDeltaV = 0;

    xReadFlag( uiSymbol );

    if ( !uiSymbol )
    {
      iPlanarDeltaU = xParsePlanarDelta( TEXT_CHROMA_U );
      iPlanarDeltaV = xParsePlanarDelta( TEXT_CHROMA_V );
    }

    pcCU->setPartSizeSubParts  ( SIZE_2Nx2N, uiAbsPartIdx, uiDepth );
    pcCU->setSizeSubParts      ( g_uiMaxCUWidth>>uiDepth, g_uiMaxCUHeight>>uiDepth, uiAbsPartIdx, uiDepth );
    pcCU->setPlanarInfoSubParts( iPlanarFlag, iPlanarDeltaY, iPlanarDeltaU, iPlanarDeltaV, uiAbsPartIdx, uiDepth );
  }
}
#endif

#if ANG_INTRA
Void TDecCavlc::parseIntraDirLumaAng  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  Int  uiIPredMode;
  Int  iMostProbable = pcCU->getMostProbableIntraDirLuma( uiAbsPartIdx );

  xReadFlag( uiSymbol );

  if ( uiSymbol )
    uiIPredMode = iMostProbable;
  else{
#if UNIFIED_DIRECTIONAL_INTRA
    Int iIntraIdx = pcCU->getIntraSizeIdx(uiAbsPartIdx);
    if ( g_aucIntraModeBitsAng[iIntraIdx] < 6 )
    {
      xReadFlag( uiSymbol ); uiIPredMode  = uiSymbol;
      if ( g_aucIntraModeBitsAng[iIntraIdx] > 2 ) { xReadFlag( uiSymbol ); uiIPredMode |= uiSymbol << 1; }
      if ( g_aucIntraModeBitsAng[iIntraIdx] > 3 ) { xReadFlag( uiSymbol ); uiIPredMode |= uiSymbol << 2; }
      if ( g_aucIntraModeBitsAng[iIntraIdx] > 4 ) { xReadFlag( uiSymbol ); uiIPredMode |= uiSymbol << 3; }
    }
    else
    {
#endif
    xReadFlag( uiSymbol ); uiIPredMode  = uiSymbol;
    xReadFlag( uiSymbol ); uiIPredMode |= uiSymbol << 1;
    xReadFlag( uiSymbol ); uiIPredMode |= uiSymbol << 2;
    xReadFlag( uiSymbol ); uiIPredMode |= uiSymbol << 3;
    xReadFlag( uiSymbol ); uiIPredMode |= uiSymbol << 4;

    if (uiIPredMode == 31){ // Escape coding for the last two modes
      xReadFlag( uiSymbol );
      uiIPredMode = uiSymbol ? 32 : 31;
    }
#if UNIFIED_DIRECTIONAL_INTRA
    }
#endif

    if (uiIPredMode >= iMostProbable)
      uiIPredMode++;
  }

  pcCU->setLumaIntraDirSubParts( (UChar)uiIPredMode, uiAbsPartIdx, uiDepth );
}
#endif

Void TDecCavlc::parseIntraDirLumaAdi  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  Int  uiIPredMode;
  Int iIntraIdx= pcCU->getIntraSizeIdx(uiAbsPartIdx);

  xReadFlag( uiSymbol );

  if ( !uiSymbol )
  {
    xReadFlag( uiSymbol );
    uiIPredMode  = uiSymbol;
    xReadFlag( uiSymbol );
    uiIPredMode |= uiSymbol << 1;
    if (g_aucIntraModeBits[iIntraIdx]>=4)
    {
      xReadFlag( uiSymbol );
      uiIPredMode |= uiSymbol << 2;
      if (g_aucIntraModeBits[iIntraIdx]>=5)
      {
        xReadFlag( uiSymbol );
        uiIPredMode |= uiSymbol << 3;
        if (g_aucIntraModeBits[iIntraIdx]>=6)
        {
          xReadFlag( uiSymbol );
          uiIPredMode |= uiSymbol << 4;
        }
      }
    }
    uiIPredMode = pcCU->revertIntraDirLumaAdi( pcCU,uiAbsPartIdx, (Int)( uiIPredMode  ) );
  }
  else
  {
    uiIPredMode  = pcCU->revertIntraDirLumaAdi( pcCU,uiAbsPartIdx, -1 );
  }
  pcCU->setLumaIntraDirSubParts( (UChar)uiIPredMode, uiAbsPartIdx, uiDepth );

  return;
}


Void TDecCavlc::parseIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;

  xReadFlag( uiSymbol );

  if ( uiSymbol )
  {
    xReadUnaryMaxSymbol( uiSymbol, 3 );
    uiSymbol++;
  }

  pcCU->setChromIntraDirSubParts( uiSymbol, uiAbsPartIdx, uiDepth );

  return ;
}

Void TDecCavlc::parseInterDir( TComDataCU* pcCU, UInt& ruiInterDir, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  
#if LCEC_PHASE2
  if(pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) <= 2 && pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) <= 2)
  {
    UInt uiIndex,uiInterDir,tmp;
    Int x,cx,y,cy;
    
    UInt vlcn = g_auiMITableVlcNum[m_uiMITableVlcIdx];
    
    UInt *m_uiMITableD = m_uiMI1TableD;
    
#ifdef QC_AMVRES
    if(pcCU->getSlice()->getSPS()->getUseAMVRes())    
      m_uiMITableD = m_uiMI2TableD;
#endif
    
    tmp = xReadVlc( vlcn );
    x = m_uiMITableD[tmp];
    uiIndex = x;
    
    /* Adapt table */
    
    cx = tmp;
    cy = Max(0,cx-1);  
    y = m_uiMITableD[cy];
    m_uiMITableD[cy] = x;
    m_uiMITableD[cx] = y;
    m_uiMITableVlcIdx += cx == m_uiMITableVlcIdx ? 0 : (cx < m_uiMITableVlcIdx ? -1 : 1);
    
#ifdef QC_AMVRES
    if(pcCU->getSlice()->getSPS()->getUseAMVRes())   
    {
      if (uiIndex<3)
        uiInterDir = 0;
      else if (uiIndex<6)
        uiInterDir = 1;
      else
        uiInterDir = 2;
      
      if (uiInterDir==0)
      {
        m_iRefFrame0[uiAbsPartIdx] = (uiIndex-0)&1;
        m_bMVres0[uiAbsPartIdx] = (uiIndex-0)==2;
      }
      else if (uiInterDir==1)
      {
        m_iRefFrame1[uiAbsPartIdx] = (uiIndex-3)&1;
        m_bMVres1[uiAbsPartIdx] = (uiIndex-3)==2;
      }
      else
      {
        m_iRefFrame0[uiAbsPartIdx] = ((uiIndex-6)/3)&1;
        m_bMVres0[uiAbsPartIdx] = ((uiIndex-6)/3)==2;
        m_iRefFrame1[uiAbsPartIdx] = ((uiIndex-6)%3)&1;
        m_bMVres1[uiAbsPartIdx] = ((uiIndex-6)%3)==2;
      }
    }
    else
#endif
    {
      uiInterDir = Min(2,uiIndex>>1);  
      if (uiInterDir==0)
        m_iRefFrame0[uiAbsPartIdx] = uiIndex&1;
      else if (uiInterDir==1)
        m_iRefFrame1[uiAbsPartIdx] = uiIndex&1;
      else
      {
        m_iRefFrame0[uiAbsPartIdx] = (uiIndex>>1)&1;
        m_iRefFrame1[uiAbsPartIdx] = (uiIndex>>0)&1;
      }
    }
    ruiInterDir = uiInterDir+1;
    return;
  }
#endif
  xReadFlag( uiSymbol );
  
  if ( uiSymbol )
  {
    uiSymbol = 2;
  }
  else
  {
    xReadFlag( uiSymbol );
  }
  uiSymbol++;
  ruiInterDir = uiSymbol;
  return;
}

Void TDecCavlc::parseRefFrmIdx( TComDataCU* pcCU, Int& riRefFrmIdx, UInt uiAbsPartIdx, UInt uiDepth, RefPicList eRefList )
{
  UInt uiSymbol;
#ifdef QC_AMVRES
  UInt uiSymbol_MVres=1;
	if(pcCU->getSlice()->getSPS()->getUseAMVRes())
	{
#if LCEC_PHASE2 //AFU
    if (pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) <= 2 && pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) <= 2)
    {
      if (eRefList==REF_PIC_LIST_0)
      {
        riRefFrmIdx = m_iRefFrame0[uiAbsPartIdx];      
      }
      if (eRefList==REF_PIC_LIST_1)
      {
         riRefFrmIdx = m_iRefFrame1[uiAbsPartIdx];
      }
      if (riRefFrmIdx==0)
      {                
        if (eRefList==REF_PIC_LIST_0)          
          uiSymbol_MVres = !m_bMVres0[uiAbsPartIdx];
        if (eRefList==REF_PIC_LIST_1)          
          uiSymbol_MVres = !m_bMVres1[uiAbsPartIdx];
	      if (!uiSymbol_MVres)
	      {
		      riRefFrmIdx = pcCU->getSlice()->getNumRefIdx( eRefList );
	      }
      }
      return;
    }    
#endif
    xReadFlag ( uiSymbol );
    if ( uiSymbol )
    {
	    xReadFlag ( uiSymbol_MVres );
	    if (uiSymbol_MVres)
	    {
		    if (pcCU->getSlice()->getNumRefIdx( eRefList )>2)
		    {
			    xReadUnaryMaxSymbol( uiSymbol, pcCU->getSlice()->getNumRefIdx( eRefList )-2 );
			    uiSymbol++;
		    }
	    }
	    else
	    {
		    uiSymbol=0;
	    }
    }
    riRefFrmIdx = uiSymbol;
	  if (!uiSymbol_MVres)
	  {
		  riRefFrmIdx = pcCU->getSlice()->getNumRefIdx( eRefList );
	  }
	}
	else
	{
#if LCEC_PHASE2 
    if (pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) <= 2 && pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) <= 2)
    {
      if (eRefList==REF_PIC_LIST_0)
      {
        riRefFrmIdx = m_iRefFrame0[uiAbsPartIdx];      
      }
      if (eRefList==REF_PIC_LIST_1)
      {
         riRefFrmIdx = m_iRefFrame1[uiAbsPartIdx];
      }
      return;
    }    
#endif
	  xReadFlag ( uiSymbol );
	  if ( uiSymbol  )
	  {
		xReadUnaryMaxSymbol( uiSymbol, pcCU->getSlice()->getNumRefIdx( eRefList )-2 );
		uiSymbol++;
	  }
	  riRefFrmIdx = uiSymbol;
	}
#else
#if LCEC_PHASE2
    if (pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_0 ) <= 2 && pcCU->getSlice()->getNumRefIdx( REF_PIC_LIST_1 ) <= 2)
    {
      if (eRefList==REF_PIC_LIST_0)
      {
        riRefFrmIdx = m_iRefFrame0[uiAbsPartIdx];      
      }
      if (eRefList==REF_PIC_LIST_1)
      {
         riRefFrmIdx = m_iRefFrame1[uiAbsPartIdx];
      }
      return;
    }    
#endif
  xReadFlag ( uiSymbol );
  if ( uiSymbol )
  {
    xReadUnaryMaxSymbol( uiSymbol, pcCU->getSlice()->getNumRefIdx( eRefList )-2 );

    uiSymbol++;
  }
  riRefFrmIdx = uiSymbol;
#endif
  return;
}
#ifdef QC_AMVRES
Void TDecCavlc::parseMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  Int iHor, iVer;
  UInt uiAbsPartIdxL, uiAbsPartIdxA;
  Int iHorPred, iVerPred;

  TComDataCU* pcCUL   = pcCU->getPULeft ( uiAbsPartIdxL, pcCU->getZorderIdxInCU() + uiAbsPartIdx );
  TComDataCU* pcCUA   = pcCU->getPUAbove( uiAbsPartIdxA, pcCU->getZorderIdxInCU() + uiAbsPartIdx );

  TComCUMvField* pcCUMvFieldL = ( pcCUL == NULL || pcCUL->isIntra( uiAbsPartIdxL ) ) ? NULL : pcCUL->getCUMvField( eRefList );
  TComCUMvField* pcCUMvFieldA = ( pcCUA == NULL || pcCUA->isIntra( uiAbsPartIdxA ) ) ? NULL : pcCUA->getCUMvField( eRefList );
  // reset mv
  TComMv cTmpMv( 0, 0 );
  pcCU->getCUMvField( eRefList )->setAllMv( cTmpMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
 
  if( pcCU->getSlice()->getSPS()->getUseAMVRes() && (pcCU->getCUMvField( eRefList )->getMVRes(uiAbsPartIdx)))
  {
		  iHorPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsHor()>>1 ) +
					 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsHor()>>1 );
		  iVerPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsVer()>>1 ) +
					 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsVer()>>1 );
		  xReadSvlc( iHor );
		  xReadSvlc( iVer );
		  iHor *=2;
		  iVer *=2;
  }
  else
  {
	  iHorPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsHor() ) +
				 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsHor() );
	  iVerPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsVer() ) +
				 ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsVer() );

	  xReadSvlc( iHor );
	  xReadSvlc( iVer );
  }
  // set mvd
  TComMv cMv( iHor, iVer );
  pcCU->getCUMvField( eRefList )->setAllMvd( cMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

  return;
}
#else
Void TDecCavlc::parseMvd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  Int iHor, iVer;
  UInt uiAbsPartIdxL, uiAbsPartIdxA;
  Int iHorPred, iVerPred;

  TComDataCU* pcCUL   = pcCU->getPULeft ( uiAbsPartIdxL, pcCU->getZorderIdxInCU() + uiAbsPartIdx );
  TComDataCU* pcCUA   = pcCU->getPUAbove( uiAbsPartIdxA, pcCU->getZorderIdxInCU() + uiAbsPartIdx );

  TComCUMvField* pcCUMvFieldL = ( pcCUL == NULL || pcCUL->isIntra( uiAbsPartIdxL ) ) ? NULL : pcCUL->getCUMvField( eRefList );
  TComCUMvField* pcCUMvFieldA = ( pcCUA == NULL || pcCUA->isIntra( uiAbsPartIdxA ) ) ? NULL : pcCUA->getCUMvField( eRefList );

  iHorPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsHor() ) +
    ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsHor() );
  iVerPred = ( (pcCUMvFieldL == NULL) ? 0 : pcCUMvFieldL->getMvd( uiAbsPartIdxL ).getAbsVer() ) +
    ( (pcCUMvFieldA == NULL) ? 0 : pcCUMvFieldA->getMvd( uiAbsPartIdxA ).getAbsVer() );

  TComMv cTmpMv( 0, 0 );
  pcCU->getCUMvField( eRefList )->setAllMv( cTmpMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

  xReadSvlc( iHor );
  xReadSvlc( iVer );

  // set mvd
  TComMv cMv( iHor, iVer );
  pcCU->getCUMvField( eRefList )->setAllMvd( cMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

  return;
}
#endif

#ifdef DCM_PBIC
Void TDecCavlc::parseMvdIcd( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth, RefPicList eRefList )
{
  Int iZeroPatt;
  Int aaiNZMv[2][2];
  Int aiNZIc[3];
  UInt uiZeroFlag;
  TComZeroTree* pcZTree;

#ifdef QC_AMVRES
  // Decode MV resolution flag (if necessary)
  Bool bMvResFlag[2] = {false, false};
  if ( pcCU->getSlice()->getSPS()->getUseAMVRes() )
  {
    if ( (eRefList == REF_PIC_LIST_0) || (eRefList == REF_PIC_LIST_X) )
      bMvResFlag[REF_PIC_LIST_0] = pcCU->getCUMvField( REF_PIC_LIST_0 )->getMVRes(uiAbsPartIdx);
    if ( (eRefList == REF_PIC_LIST_1) || (eRefList == REF_PIC_LIST_X) )
      bMvResFlag[REF_PIC_LIST_1] = pcCU->getCUMvField( REF_PIC_LIST_1 )->getMVRes(uiAbsPartIdx);
  }
#endif

  // Is any component non-zero?
  xReadFlag( uiZeroFlag );
  if (uiZeroFlag)
  {
    TComMv cZeroMv;
    if (eRefList == REF_PIC_LIST_X)
    {
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cZeroMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cZeroMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
    }
    else
      pcCU->getCUMvField(       eRefList )->setAllMvd( cZeroMv, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      TComIc cDefaultIc;
      pcCU->getCUIcField()->setAllIcd( cDefaultIc, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
    }
    return;
  }

  // Identify the non-zero components
  if (eRefList == REF_PIC_LIST_X)
  {
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      pcZTree   = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDICDBI);
      iZeroPatt = parseZTree( pcZTree );
    }
    else
    {
      pcZTree   = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDBI);
      iZeroPatt = parseZTree( pcZTree );
    }
    aaiNZMv[REF_PIC_LIST_0][0] = iZeroPatt & 1;
    aaiNZMv[REF_PIC_LIST_0][1] = iZeroPatt & 2;
    aaiNZMv[REF_PIC_LIST_1][0] = iZeroPatt & 4;
    aaiNZMv[REF_PIC_LIST_1][1] = iZeroPatt & 8;
    aiNZIc[0] = iZeroPatt & 16;
    aiNZIc[1] = iZeroPatt & 32;
    aiNZIc[2] = iZeroPatt & 64;
  }
  else
  {
    if (pcCU->getSlice()->getSPS()->getUseIC())
    {
      pcZTree   = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDICDUNI);
      iZeroPatt = parseZTree( pcZTree );
    }
    else
    {
      pcZTree   = pcCU->getSlice()->getZTree(IDX_ZTREE_MVDUNI);
      iZeroPatt = parseZTree( pcZTree );
    }
    aaiNZMv[eRefList  ][0] = iZeroPatt & 1;
    aaiNZMv[eRefList  ][1] = iZeroPatt & 2;
    aaiNZMv[eRefList^1][0] = 0;
    aaiNZMv[eRefList^1][1] = 0;
    aiNZIc[0] = iZeroPatt & 4;
    aiNZIc[1] = 0;
    aiNZIc[2] = iZeroPatt & 8;
  }

  //Decode the non-zero components
  TComMv cMvd0, cMvd1;
  Int iHor, iVer;

  iHor = iVer = 0;
  if (aaiNZMv[REF_PIC_LIST_0][0]) xReadSvlcNZ(iHor);
  if (aaiNZMv[REF_PIC_LIST_0][1]) xReadSvlcNZ(iVer);
#ifdef QC_AMVRES
  if (bMvResFlag[REF_PIC_LIST_0] == true)
  {
    iHor *= 2;
    iVer *= 2;
  }
#endif
  cMvd0.set( iHor, iVer );

  iHor = iVer = 0;
  if (aaiNZMv[REF_PIC_LIST_1][0]) xReadSvlcNZ(iHor);
  if (aaiNZMv[REF_PIC_LIST_1][1]) xReadSvlcNZ(iVer);
#ifdef QC_AMVRES
  if (bMvResFlag[REF_PIC_LIST_1] == true)
  {
    iHor *= 2;
    iVer *= 2;
  }
#endif
  cMvd1.set( iHor, iVer );

  if (eRefList == REF_PIC_LIST_X)
  {
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvd0, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cMvd1, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
  }
  else if (eRefList == REF_PIC_LIST_1)
    pcCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvd( cMvd1, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
  else
    pcCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvd( cMvd0, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );

  if (pcCU->getSlice()->getSPS()->getUseIC())
  {
    TComIc cIcd;
    Int iParam0, iParam1, iParam2;
    iParam0 = iParam1 = iParam2 = 0;
    if (aiNZIc[0]) xReadSvlcNZ(iParam0);
    if (aiNZIc[1]) xReadSvlcNZ(iParam1);
    if (aiNZIc[2]) xReadSvlcNZ(iParam2);
    cIcd.setIcParam( iParam0, iParam1, iParam2 );
    pcCU->getCUIcField()->setAllIcd( cIcd, pcCU->getPartitionSize( uiAbsPartIdx ), uiAbsPartIdx, uiPartIdx, uiDepth );
  }
}

Int TDecCavlc::parseZTree(TComZeroTree* pcZTree)
{
  Int iVal;
  Int iStack;
  Int iResult;
  Int iIsNotLeaf;
  UInt uiNodeBoth;
  UInt uiNodeLeft;
  UInt uiCtx;
  Int* piZTreeStructure = pcZTree->m_piStructure;
  Int* piLeafIdx        = pcZTree->m_piLeafIdx;

  iVal    = 1;
  iStack  = 0;
  iResult = 0;
  uiCtx   = 0;

  while (true)
  {
    STATE_1:
    iIsNotLeaf = *piZTreeStructure++;

    if (iIsNotLeaf == 0)
    {
      iResult += (1 << (*piLeafIdx));

      if (iStack == 0)
        break;

      piLeafIdx++;
      iVal = iStack & 1;
      iStack >>= 1;

      if (iVal == 0)
        goto STATE_0;
    }
    else
    {
      iStack <<= 1;

      xReadFlag( uiNodeBoth );
      if (uiNodeBoth)
        iStack++;
      else
      {
        xReadFlag( uiNodeLeft );
        if (uiNodeLeft == 0)
        {
          iStack++;
          uiCtx += 2;
          goto STATE_0;
        }
      }

      uiCtx += 2;
    }

    goto STATE_1;


    STATE_0:
    iIsNotLeaf = *piZTreeStructure++;

    if (iIsNotLeaf == 0)
    {
      if (iStack == 0)
        break;

      piLeafIdx++;
      iVal = iStack & 1;
      iStack >>= 1;

      if (iVal != 0)
        goto STATE_1;

    }
    else
    {
      iStack <<= 1;
      uiCtx += 2;
    }

    goto STATE_0;
  }

  return iResult;
}

ContextModel* TDecCavlc::getZTreeCtx ( Int iIdx )
{
  return NULL;
}
#endif

Void TDecCavlc::parseTransformIdx( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiTrLevel = 0;

  UInt uiWidthInBit  = g_aucConvertToBit[pcCU->getWidth(uiAbsPartIdx)]+2;
  UInt uiTrSizeInBit = g_aucConvertToBit[pcCU->getSlice()->getSPS()->getMaxTrSize()]+2;
  uiTrLevel          = uiWidthInBit >= uiTrSizeInBit ? uiWidthInBit - uiTrSizeInBit : 0;

  UInt uiMinTrDepth = pcCU->getSlice()->getSPS()->getMinTrDepth() + uiTrLevel;
  UInt uiMaxTrDepth = pcCU->getSlice()->getSPS()->getMaxTrDepth() + uiTrLevel;

  if ( uiMinTrDepth == uiMaxTrDepth )
  {
    pcCU->setTrIdxSubParts( uiMinTrDepth, uiAbsPartIdx, uiDepth );
    return;
  }

  UInt uiTrIdx;
  xReadFlag( uiTrIdx );

  if ( !uiTrIdx )
  {
    uiTrIdx = uiTrIdx + uiMinTrDepth;
    pcCU->setTrIdxSubParts( uiTrIdx, uiAbsPartIdx, uiDepth );
    return;
  }

  if (pcCU->getPartitionSize(uiAbsPartIdx) >= SIZE_2NxnU && pcCU->getPartitionSize(uiAbsPartIdx) <= SIZE_nRx2N && uiMinTrDepth == 0 && uiMaxTrDepth == 1)
  {
    uiTrIdx++;

    ///Maybe unnecessary///
    UInt      uiWidth      = pcCU->getWidth ( uiAbsPartIdx );
    while((uiWidth>>uiTrIdx) < (g_uiMaxCUWidth>>g_uiMaxCUDepth)) uiTrIdx--;
    ////////////////////////

    uiTrIdx = uiTrIdx + uiMinTrDepth;
    pcCU->setTrIdxSubParts( uiTrIdx, uiAbsPartIdx, uiDepth );
    return;
  }

  UInt uiSymbol;
  Int  iCount = 1;
  while( ++iCount <= (Int)( uiMaxTrDepth - uiMinTrDepth ) )
  {
    xReadFlag( uiSymbol );
    if ( uiSymbol == 0 )
    {
      uiTrIdx = uiTrIdx + uiMinTrDepth;
      pcCU->setTrIdxSubParts( uiTrIdx, uiAbsPartIdx, uiDepth );
      return;
    }
    uiTrIdx += uiSymbol;
  }

  uiTrIdx = uiTrIdx + uiMinTrDepth;

  pcCU->setTrIdxSubParts( uiTrIdx, uiAbsPartIdx, uiDepth );

  return ;
}

Void TDecCavlc::parseDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiDQp;
  Int  iDQp;

  xReadFlag( uiDQp );

  if ( uiDQp == 0 )
  {
    uiDQp = pcCU->getSlice()->getSliceQp();
  }
  else
  {
    xReadSvlc( iDQp );
    uiDQp = pcCU->getSlice()->getSliceQp() + iDQp;
  }

  pcCU->setQPSubParts( uiDQp, uiAbsPartIdx, uiDepth );
}

Void TDecCavlc::parseCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth, UInt uiDepth )
{
#if HHI_RQT
  if( pcCU->getSlice()->getSPS()->getQuadtreeTUFlag() )
  {
#if HHI_RQT_INTRA
    return;
#else
    if( !pcCU->isIntra( uiAbsPartIdx ) )
    {
      return;
    }
#endif
  }
#endif

#if LCEC_PHASE2
  if (eType == TEXT_ALL)
  {
    UInt uiCbf,tmp;
    UInt uiCBP,uiCbfY,uiCbfU,uiCbfV;
    Int n,x,cx,y,cy;

    /* Start adaptation */
    n = pcCU->isIntra( uiAbsPartIdx ) ? 0 : 1;
    UInt vlcn = g_auiCbpVlcNum[n][m_uiCbpVlcIdx[n]];
    tmp = xReadVlc( vlcn );    
    uiCBP = m_uiCBPTableD[n][tmp];
    
    /* Adapt LP table */
    cx = tmp;
    cy = Max(0,cx-1);
    x = uiCBP;
    y = m_uiCBPTableD[n][cy];
    m_uiCBPTableD[n][cy] = x;
    m_uiCBPTableD[n][cx] = y;
    m_uiCbpVlcIdx[n] += cx == m_uiCbpVlcIdx[n] ? 0 : (cx < m_uiCbpVlcIdx[n] ? -1 : 1);

    uiCbfY = (uiCBP>>0)&1;
    uiCbfU = (uiCBP>>1)&1;
    uiCbfV = (uiCBP>>2)&1;

    uiCbf = pcCU->getCbf( uiAbsPartIdx, TEXT_LUMA );
    pcCU->setCbfSubParts( uiCbf | ( uiCbfY << uiTrDepth ), TEXT_LUMA, uiAbsPartIdx, uiDepth );
    
    uiCbf = pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_U );
    pcCU->setCbfSubParts( uiCbf | ( uiCbfU << uiTrDepth ), TEXT_CHROMA_U, uiAbsPartIdx, uiDepth );
        
    uiCbf = pcCU->getCbf( uiAbsPartIdx, TEXT_CHROMA_V );
    pcCU->setCbfSubParts( uiCbf | ( uiCbfV << uiTrDepth ), TEXT_CHROMA_V, uiAbsPartIdx, uiDepth );
    return;
  }
  
#endif

  UInt uiSymbol;
  UInt uiCbf = pcCU->getCbf( uiAbsPartIdx, eType );

  xReadFlag( uiSymbol );
  pcCU->setCbfSubParts( uiCbf | ( uiSymbol << uiTrDepth ), eType, uiAbsPartIdx, uiDepth );

  return;
}

Void TDecCavlc::parseCoeffNxN( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType )
{

  if ( uiWidth > pcCU->getSlice()->getSPS()->getMaxTrSize() )
  {
    uiWidth  = pcCU->getSlice()->getSPS()->getMaxTrSize();
    uiHeight = pcCU->getSlice()->getSPS()->getMaxTrSize();
  }
  UInt uiSize   = uiWidth*uiHeight;

  // point to coefficient
  TCoeff* piCoeff = pcCoef;

  // initialize scan
  const UInt*  pucScan;

#if LCEC_PHASE1
  //UInt uiConvBit = g_aucConvertToBit[ Min(8,uiWidth) ];
  UInt uiConvBit = g_aucConvertToBit[ pcCU->isIntra( uiAbsPartIdx ) ? uiWidth : Min(8,uiWidth)    ];
#else
  UInt uiConvBit = g_aucConvertToBit[ uiWidth    ];
#endif
#if HHI_RQT
  pucScan        = g_auiFrameScanXY  [ uiConvBit + 1 ];
#else
  pucScan        = g_auiFrameScanXY  [ uiConvBit ];
#endif

#if QC_MDDT//VLC_MDDT ADAPTIVE_SCAN
  UInt *scanStats;
  UInt uiMode, uiPredMode;
	int indexROT;
  if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA && (uiWidth == 4 || uiWidth == 8 || uiWidth==16 || uiWidth==32 || uiWidth==64))
  {
    uiMode = pcCU->getLumaIntraDir(uiAbsPartIdx);
	indexROT = pcCU->getROTindex(uiAbsPartIdx);
	int scan_index;
    if(uiWidth == 4)// && ipredmode<=8&&indexROT == 0)
    {
      uiPredMode = g_aucIntra9Mode[uiMode];
       pucScan = scanOrder4x4[uiPredMode]; //pucScanX = scanOrder4x4X[ipredmode]; pucScanY = scanOrder4x4Y[ipredmode];

       scanStats = scanStats4x4[uiPredMode]; update4x4Count[uiPredMode]++;
    }
    else if(uiWidth == 8)// && ipredmode<=8 && indexROT == 0)
    {
      uiPredMode = ((1 << (pcCU->getIntraSizeIdx( uiAbsPartIdx ) + 1)) != uiWidth) ?  g_aucIntra9Mode[uiMode]: g_aucAngIntra9Mode[uiMode];
      pucScan = scanOrder8x8[uiPredMode]; //pucScanX = scanOrder8x8X[ipredmode]; pucScanY = scanOrder8x8Y[ipredmode];
 
      scanStats = scanStats8x8[uiPredMode]; update8x8Count[uiPredMode]++;
    }
	else if(uiWidth == 16)
    {
	  scan_index = LUT16x16[indexROT][uiMode];
      pucScan = scanOrder16x16[scan_index]; //pucScanX = scanOrder16x16X[scan_index]; pucScanY = scanOrder16x16Y[scan_index];
      scanStats = scanStats16x16[scan_index];
    }
    else if(uiWidth == 32)
    {
	  scan_index = LUT32x32[indexROT][uiMode];
      pucScan = scanOrder32x32[scan_index]; //pucScanX = scanOrder32x32X[scan_index]; pucScanY = scanOrder32x32Y[scan_index];
      scanStats = scanStats32x32[scan_index];
    }
    else if(uiWidth == 64)
    {
	  scan_index = LUT64x64[indexROT][uiMode];
      pucScan = scanOrder64x64[scan_index]; //pucScanX = scanOrder64x64X[scan_index]; pucScanY = scanOrder64x64Y[scan_index];
      scanStats = scanStats64x64[scan_index];
    }
    else
    {
      //printf("uiWidth = %d is not supported!\n", uiWidth);
      //exit(1);
    }
  }
#endif

#if LCEC_PHASE1
  UInt uiDecodeDCCoeff = 0;
  Int dcCoeff = 0;
  if (pcCU->isIntra(uiAbsPartIdx))
  {
    UInt uiAbsPartIdxL, uiAbsPartIdxA;
    TComDataCU* pcCUL   = pcCU->getPULeft (uiAbsPartIdxL, pcCU->getZorderIdxInCU() + uiAbsPartIdx);
    TComDataCU* pcCUA   = pcCU->getPUAbove(uiAbsPartIdxA, pcCU->getZorderIdxInCU() + uiAbsPartIdx);
	if (pcCUL == NULL && pcCUA == NULL)
	{
#if 1
	  uiDecodeDCCoeff = 1;
      dcCoeff = xReadVlc(eTType == TEXT_LUMA ? 3 : 1);
      if (dcCoeff)
      {
	    UInt sign;
        xReadFlag(sign);
        if (sign)
        {
          dcCoeff = -dcCoeff;
        }
	  }
#else
      UInt sign;
        xReadFlag(sign);
#endif
	}
  }
#endif

  UInt uiScanning;
#if !QC_MDDT && !LCEC_PHASE1_ADAPT_ENABLE
  UInt uiInterleaving, uiIsCoded;
#endif

  TCoeff scoeff[64];
  Int iBlockType;
#if HHI_RQT
  if( uiSize == 2*2 )
  {
    // hack: re-use 4x4 coding
    iBlockType = pcCU->isIntra(uiAbsPartIdx) ? 0 : pcCU->getSlice()->getSliceType();
    xParseCoeff4x4( scoeff, iBlockType );

    for (uiScanning=0; uiScanning<4; uiScanning++)
    {
      piCoeff[ pucScan[ uiScanning ] ] = scoeff[15-uiScanning];
    }
  }
  else if ( uiSize == 4*4 )
#else
  if ( uiSize == 4*4 )
#endif
  {
    iBlockType = pcCU->isIntra(uiAbsPartIdx) ? 0 : pcCU->getSlice()->getSliceType();
    xParseCoeff4x4( scoeff, iBlockType );

    for (uiScanning=0; uiScanning<16; uiScanning++)
    {
      piCoeff[ pucScan[ uiScanning ] ] = scoeff[15-uiScanning];

#if QC_MDDT//VLC_MDDT ADAPTIVE_SCAN
      if(scoeff[15-uiScanning])
      {
        if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA)//  && (uiWidth == 4 && ipredmode<=8&&indexROT == 0))
        {
          //scanStats[pucScan[ui]]++;
          scanStats[uiScanning]++;
        }
      }
#endif
    }
  }
  else if ( uiSize == 8*8 )
  {
#if LCEC_PHASE1
    if (eTType==TEXT_CHROMA_U || eTType==TEXT_CHROMA_V) //8x8 specific
      iBlockType = eTType-2;
    else
#endif
    iBlockType = 2 + ( pcCU->isIntra(uiAbsPartIdx) ? 0 : pcCU->getSlice()->getSliceType() );
    xParseCoeff8x8( scoeff, iBlockType );

    for (uiScanning=0; uiScanning<64; uiScanning++)
    {
      piCoeff[ pucScan[ uiScanning ] ] = scoeff[63-uiScanning];

#if QC_MDDT//VLC_MDDT ADAPTIVE_SCAN
      if(scoeff[63-uiScanning])
      {
        if(pcCU->isIntra( uiAbsPartIdx ) && eTType == TEXT_LUMA)//  && (uiWidth == 8 && ipredmode<=8 && indexROT == 0))
        {
          //scanStats[pucScan[ui]]++;
          scanStats[uiScanning]++;
        }
      }
#endif
    }

  }
  else
  {
#if LCEC_PHASE1
    if (!pcCU->isIntra( uiAbsPartIdx ))
    {
	    memset(piCoeff,0,sizeof(TCoeff)*uiSize);
      if (eTType==TEXT_CHROMA_U || eTType==TEXT_CHROMA_V) 
        iBlockType = eTType-2;
      else
        iBlockType = 5 + ( pcCU->isIntra(uiAbsPartIdx) ? 0 : pcCU->getSlice()->getSliceType() );
      xParseCoeff8x8( scoeff, iBlockType );

      for (uiScanning=0; uiScanning<64; uiScanning++)
      {	  
        piCoeff[(pucScan[uiScanning]/8)*uiWidth + (pucScan[uiScanning]%8)] = scoeff[63-uiScanning];
      }
      return;
    }
#endif

#if QC_MDDT
    if(pcCU->isIntra( uiAbsPartIdx ))
    {
	    memset(piCoeff,0,sizeof(TCoeff)*uiSize);

      if (eTType==TEXT_CHROMA_U || eTType==TEXT_CHROMA_V) 
        iBlockType = eTType-2;
      else
        iBlockType = 5 + ( pcCU->isIntra(uiAbsPartIdx) ? 0 : pcCU->getSlice()->getSliceType() );
      xParseCoeff8x8( scoeff, iBlockType );

      for (uiScanning=0; uiScanning<64; uiScanning++)
      {
        if(piCoeff[ pucScan[ uiScanning ] ] = scoeff[63-uiScanning])
        {
          if(eTType == TEXT_LUMA)
            scanStats[ uiScanning ]++;
        }
      }
    }
#else
#if LCEC_PHASE1_ADAPT_ENABLE
    if(pcCU->isIntra( uiAbsPartIdx ))
    {
	    memset(piCoeff,0,sizeof(TCoeff)*uiSize);

      if (eTType==TEXT_CHROMA_U || eTType==TEXT_CHROMA_V) 
        iBlockType = eTType-2;
      else
        iBlockType = 5 + ( pcCU->isIntra(uiAbsPartIdx) ? 0 : pcCU->getSlice()->getSliceType() );
      xParseCoeff8x8( scoeff, iBlockType );

      for (uiScanning=0; uiScanning<64; uiScanning++)
      {
        piCoeff[ pucScan[ uiScanning ] ] = scoeff[63-uiScanning];
      }
    }
#else
    for (uiInterleaving=0; uiInterleaving<uiSize/64; uiInterleaving++)
    {
      xReadFlag( uiIsCoded );
      if ( !uiIsCoded )
      {
        for (uiScanning=0; uiScanning<64; uiScanning++)
        {
          piCoeff[ pucScan[ (uiSize/64) * uiScanning + uiInterleaving ] ] = 0;
        }
      }
      else
      {
        iBlockType = 5 + ( pcCU->isIntra(uiAbsPartIdx) ? 0 : pcCU->getSlice()->getSliceType() );
        xParseCoeff8x8( scoeff, iBlockType );

        for (uiScanning=0; uiScanning<64; uiScanning++)
        {
          piCoeff[ pucScan[ (uiSize/64) * uiScanning + uiInterleaving ] ] = scoeff[63-uiScanning];
        }
      }
    }
#endif
#endif // QC_MDDT
//#endif
  }

#if LCEC_PHASE1
  if (uiDecodeDCCoeff == 1)
  {
    piCoeff[0] = dcCoeff;
  }
#endif

  return ;
}

#if HHI_RQT
Void TDecCavlc::parseTransformSubdivFlag( UInt& ruiSubdivFlag, UInt uiLog2TransformBlockSize )
{
  xReadFlag( ruiSubdivFlag );
}

Void TDecCavlc::parseQtCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth, UInt uiDepth )
{
  UInt uiSymbol;
  xReadFlag( uiSymbol );
  pcCU->setCbfSubParts( uiSymbol << uiTrDepth, eType, uiAbsPartIdx, uiDepth );
}

#if HHI_RQT_ROOT
Void TDecCavlc::parseQtRootCbf( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt& uiQtRootCbf )
{
  UInt uiSymbol;
  xReadFlag( uiSymbol );
  uiQtRootCbf = uiSymbol;
}
#endif
#endif

Void TDecCavlc::parseAlfFlag (UInt& ruiVal)
{
  xReadFlag( ruiVal );
}

#if TSB_ALF_HEADER
Void TDecCavlc::parseAlfFlagNum( UInt& ruiVal, UInt minValue, UInt depth )
{
  UInt uiLength = 0;
  UInt maxValue = (minValue << (depth*2));
  UInt temp = maxValue - minValue;
  for(UInt i=0; i<32; i++)
  {
    if(temp&0x1)
    {
      uiLength = i+1;
    }
    temp = (temp >> 1);
  }
  if(uiLength)
  {
    xReadCode( uiLength, ruiVal );
  }
  else
  {
    ruiVal = 0;
  }
  ruiVal += minValue;
}

Void TDecCavlc::parseAlfCtrlFlag( UInt &ruiAlfCtrlFlag )
{
  UInt uiSymbol;
  xReadFlag( uiSymbol );
  ruiAlfCtrlFlag = uiSymbol;
}
#endif

Void TDecCavlc::parseAlfUvlc (UInt& ruiVal)
{
  xReadUvlc( ruiVal );
}

Void TDecCavlc::parseAlfSvlc (Int&  riVal)
{
  xReadSvlc( riVal );
}

#if HHI_ALF
Void TDecCavlc::parseAlfQTCtrlFlag   ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  if (!m_bAlfCtrl)
    return;

  UInt uiSymbol;
  xReadFlag( uiSymbol );

  if (uiDepth > m_uiMaxAlfCtrlDepth)
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, m_uiMaxAlfCtrlDepth);
  }
  else
  {
    pcCU->setAlfCtrlFlagSubParts( uiSymbol, uiAbsPartIdx, uiDepth );
  }
}

Void TDecCavlc::parseAlfQTSplitFlag  ( TComDataCU* pcCU ,UInt uiAbsPartIdx, UInt uiDepth, UInt uiMaxDepth )
{
  //if( uiDepth >= uiMaxDepth )
  if( uiDepth >= g_uiMaxCUDepth-g_uiAddCUDepth ) // fix HS
  {
    pcCU->setDepthSubParts( uiDepth, uiAbsPartIdx );
    return;
  }

  UInt uiSymbol;
  xReadFlag( uiSymbol );
  pcCU->setDepthSubParts( uiDepth + uiSymbol, uiAbsPartIdx );

  return;
}

Void TDecCavlc::parseAlfDc( Int& riDc    )
{
  UInt uiCode;
  UInt iSign ;

  xReadFlag( uiCode );
  if ( uiCode == 0)
  {
    riDc = 0 ;
    return ;
  }

  xReadFlag( uiCode );
  if ( uiCode == 0 )  iSign =  1;
  else                iSign = -1;

  xReadEpExGolomb(uiCode, 9);
  riDc = uiCode*iSign ;
}

Void TDecCavlc::parseAlfCoeff( Int& riCoeff, Int iLength, Int iPos )
{
  UInt uiCode;
  UInt iSign ;

  xReadFlag( uiCode ) ;
  if ( uiCode == 0)
  {
    riCoeff = 0 ;
    return ;
  }

  xReadFlag( uiCode );
  if ( uiCode == 0 )  iSign =  1;
  else                iSign = -1;

  Int iM =4;
  if(iLength==3)
  {
    if(iPos == 0)
      iM = 4 ;
    else if(iPos == 1)
      iM = 1 ;
    else if(iPos == 2)
      iM = 2 ;
  }
  else if(iLength==5)
  {
    if(iPos == 0)
      iM = 3 ;
    else if(iPos == 1)
      iM = 5 ;
    else if(iPos == 2)
      iM = 1 ;
    else if(iPos == 3)
      iM = 3 ;
    else if(iPos == 4)
      iM = 2 ;
  }
  else if(iLength==7)
  {
    if(iPos == 0)
      iM = 3 ;
    else if(iPos == 1)
      iM = 4 ;
    else if(iPos == 2)
      iM = 5;
    else if(iPos == 3)
      iM = 1 ;
    else if(iPos == 4)
      iM = 3 ;
    else if(iPos == 5)
     iM = 3 ;
    else if(iPos == 6)
     iM = 2 ;
  }
  else if(iLength==9)
  {
    if(iPos == 0)
      iM = 2 ;
    else if(iPos == 1)
      iM = 4 ;
    else if(iPos == 2)
      iM = 4 ;
    else if(iPos == 3)
      iM = 5 ;
    else if(iPos == 4)
      iM = 1 ;
    else if(iPos == 5)
     iM = 3 ;
    else if(iPos == 6)
     iM = 3 ;
    else if(iPos == 7)
    iM = 3 ;
   else if(iPos == 8)
    iM = 2 ;
  }

  xReadEpExGolomb(uiCode, iM);
  riCoeff = uiCode*iSign ;
}
#endif

#if HHI_MRG
#if HHI_MRG_PU
Void TDecCavlc::parseMergeFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx )
{
  UInt uiSymbol;
  UInt uiCtxIdx = pcCU->getCtxMergeFlag( uiAbsPartIdx );
  xReadFlag( uiSymbol );
  pcCU->setMergeFlagSubParts( uiSymbol ? true : false, uiAbsPartIdx, uiPUIdx, uiDepth );
}

Void TDecCavlc::parseMergeIndex ( TComDataCU* pcCU, UInt& ruiMergeIndex, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  UInt uiCtxIdx = pcCU->getCtxMergeIndex( uiAbsPartIdx );
  xReadFlag( uiSymbol );
  ruiMergeIndex = uiSymbol;
}
#else
Void TDecCavlc::parseMergeFlag ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  xReadFlag( uiSymbol );
  pcCU->setMergeFlagSubParts( uiSymbol ? true : false, uiAbsPartIdx, uiDepth );
}

Void TDecCavlc::parseMergeIndex ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt uiSymbol;
  xReadFlag( uiSymbol );
  pcCU->setMergeIndexSubParts( uiSymbol, uiAbsPartIdx, uiDepth );
}
#endif
#endif

#if HHI_AIS
Void TDecCavlc::parseIntraFiltFlagLumaAdi ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  UInt  uiSymbol;
  xReadFlag( uiSymbol );

  //pcCU->setLumaIntraFiltFlag( uiAbsPartIdx, uiSymbol ? true : false );
  pcCU->setLumaIntraFiltFlagSubParts( uiSymbol != 0, uiAbsPartIdx, uiDepth ); // fix HS
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Void TDecCavlc::xReadCode (UInt uiLength, UInt& ruiCode)
{
  assert ( uiLength > 0 );
  m_pcBitstream->read (uiLength, ruiCode);
}

Void TDecCavlc::xReadUvlc( UInt& ruiVal)
{
  UInt uiVal = 0;
  UInt uiCode = 0;
  UInt uiLength;
  m_pcBitstream->read( 1, uiCode );

  if( 0 == uiCode )
  {
    uiLength = 0;

    while( ! ( uiCode & 1 ))
    {
      m_pcBitstream->read( 1, uiCode );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiVal );

    uiVal += (1 << uiLength)-1;
  }

  ruiVal = uiVal;
}

Void TDecCavlc::xReadSvlc( Int& riVal)
{
  UInt uiBits = 0;
  m_pcBitstream->read( 1, uiBits );
  if( 0 == uiBits )
  {
    UInt uiLength = 0;

    while( ! ( uiBits & 1 ))
    {
      m_pcBitstream->read( 1, uiBits );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiBits );

    uiBits += (1 << uiLength);
    riVal = ( uiBits & 1) ? -(Int)(uiBits>>1) : (Int)(uiBits>>1);
  }
  else
  {
    riVal = 0;
  }
}

#ifdef DCM_PBIC
Void TDecCavlc::xReadSvlcNZ( Int& riVal)
{
  UInt uiCode, uiSign;
  xReadUvlc( uiCode );
  uiCode++;
  xReadFlag( uiSign );
  riVal = (uiSign == 1) ? -Int(uiCode): Int(uiCode);
}
#endif

Void TDecCavlc::xReadFlag (UInt& ruiCode)
{
  m_pcBitstream->read( 1, ruiCode );
}

Void TDecCavlc::xReadUnaryMaxSymbol( UInt& ruiSymbol, UInt uiMaxSymbol )
{
  xReadFlag( ruiSymbol );

  if (ruiSymbol == 0 || uiMaxSymbol == 1)
  {
    return;
  }

  UInt uiSymbol = 0;
  UInt uiCont;

  do
  {
    xReadFlag( uiCont );
    uiSymbol++;
  }
  while( uiCont && (uiSymbol < uiMaxSymbol-1) );

  if( uiCont && (uiSymbol == uiMaxSymbol-1) )
  {
    uiSymbol++;
  }

  ruiSymbol = uiSymbol;
}

Void TDecCavlc::xReadExGolombLevel( UInt& ruiSymbol )
{
  UInt uiSymbol ;
  UInt uiCount = 0;
  do
  {
    xReadFlag( uiSymbol );
    uiCount++;
  }
  while( uiSymbol && (uiCount != 13));

  ruiSymbol = uiCount-1;

  if( uiSymbol )
  {
    xReadEpExGolomb( uiSymbol, 0 );
    ruiSymbol += uiSymbol+1;
  }

  return;
}

Void TDecCavlc::xReadEpExGolomb( UInt& ruiSymbol, UInt uiCount )
{
  UInt uiSymbol = 0;
  UInt uiBit = 1;


  while( uiBit )
  {
    xReadFlag( uiBit );
    uiSymbol += uiBit << uiCount++;
  }

  uiCount--;
  while( uiCount-- )
  {
    xReadFlag( uiBit );
    uiSymbol += uiBit << uiCount;
  }

  ruiSymbol = uiSymbol;

  return;
}

UInt TDecCavlc::xGetBit()
{
  UInt ruiCode;
  m_pcBitstream->read( 1, ruiCode );
  return ruiCode;
}

Int TDecCavlc::xReadVlc( Int n )
{
  assert( n>=0 && n<=10 );

  UInt zeroes=0, done=0, tmp;
  UInt cw, bit;
  UInt val = 0;
  UInt first;
  UInt lead = 0;

  if (n < 5)
  {
    while (!done && zeroes < 6)
    {
      xReadFlag( bit );
      if (bit)
      {
        if (n)
        {
          xReadCode( n, cw );
        }
        else
        {
          cw = 0;
        }
        done = 1;
      }
      else
      {
        zeroes++;
      }
    }
    if ( done )
    {
      val = (zeroes<<n)+cw;
    }
    else
    {
      lead = n;
      while (!done)
      {
        xReadFlag( first );
        if ( !first )
        {
          lead++;
        }
        else
        {
          if ( lead )
          {
            xReadCode( lead, tmp );
          }
          else
          {
            tmp = 0;
          }
          val = 6 * (1 << n) + (1 << lead) + tmp - (1 << n);
          done = 1;
        }
      }
    }
  }
  else if (n < 8)
  {
    while (!done)
    {
      xReadFlag( bit );
      if ( bit )
      {
        xReadCode( n-4, cw );
        done = 1;
      }
      else
      {
        zeroes++;
      }
    }
    val = (zeroes<<(n-4))+cw;
  }
  else if (n == 8)
  {
    if ( xGetBit() )
    {
      val = 0;
    }
    else if ( xGetBit() )
    {
      val = 1;
    }
    else
    {
      val = 2;
    }
  }
  else if (n == 9)
  {
    if ( xGetBit() )
    {
      if ( xGetBit() )
      {
        xReadCode(3, val);
        val += 3;
      }
      else if ( xGetBit() )
      {
        val = xGetBit() + 1;
      }
      else
      {
        val = 0;
      }
    }
    else
    {
      while (!done)
      {
        xReadFlag( bit );
        if ( bit )
        {
          xReadCode(4, cw);
          done = 1;
        }
        else
        {
          zeroes++;
        }
      }
      val = (zeroes<<4)+cw+11;
    }
  }
  else if (n == 10)
  {
    while (!done)
    {
      xReadFlag( first );
      if ( !first )
      {
        lead++;
      }
      else
      {
        if ( !lead )
        {
          val = 0;
        }
        else
        {
          xReadCode(lead, val);
          val += (1<<lead);
          val--;
        }
        done = 1;
      }
    }
  }
  return val;
}

Void TDecCavlc::xParseCoeff4x4( TCoeff* scoeff, Int n )
{
  Int i;
  UInt sign;
  Int tmp;
  Int vlc,cn,this_pos;
  Int maxrun;
  Int last_position;
  Int atable[5] = {4,6,14,28,0xfffffff};
  Int vlc_adaptive=0;
  Int done;
  LastCoeffStruct combo;

  for (i = 0; i < 16; i++)
  {
    scoeff[i] = 0;
  }

  {
    /* Get the last nonzero coeff */
    Int x,y,cx,cy,vlcNum;
    Int vlcTable[8] = {2,2,2};

    /* Decode according to current LP table */
    vlcNum = vlcTable[n];

    tmp = xReadVlc( vlcNum );
    cn = m_uiLPTableD4[n][tmp];
    combo.level = (cn>15);
    combo.last_pos = cn&0x0f;

    /* Adapt LP table */
    cx = tmp;
    cy = Max( 0, cx-1 );
    x = cn;
    y = m_uiLPTableD4[n][cy];
    m_uiLPTableD4[n][cy] = x;
    m_uiLPTableD4[n][cx] = y;
  }

  if ( combo.level == 1 )
  {
    tmp = xReadVlc( 0 );
    sign = tmp&1;
    tmp = (tmp>>1)+2;
  }
  else
  {
    tmp = 1;
    xReadFlag( sign );
  }

  if ( sign )
  {
    tmp = -tmp;
  }

  last_position = combo.last_pos;
  this_pos = 15 - last_position;
  scoeff[this_pos] = tmp;
  i = this_pos;
  i++;

  done = 0;
  {
    while (!done && i < 16)
    {
      maxrun = 15-i;
      if (maxrun > 27)
      {
        maxrun = 28;
        vlc = 3;
      }
      else
      {
        vlc = g_auiVlcTable8x8[maxrun];
      }

      /* Go into run mode */
      cn = xReadVlc( vlc );
      combo = g_acstructLumaRun8x8[maxrun][cn];
      i += combo.last_pos;
      /* No sign for last zeroes */
      if (i < 16)
      {
        if (combo.level == 1)
        {
          tmp = xReadVlc( 0 );
          sign = tmp&1;
          tmp = (tmp>>1)+2;
          done = 1;
        }
        else
        {
          tmp = 1;
          xReadFlag( sign );
        }
        if ( sign )
        {
          tmp = -tmp;
        }
        scoeff[i] = tmp;
      }
      i++;
    }
  }
  if (i < 16)
  {
    /* Get the rest in level mode */
    while ( i < 16 )
    {
      tmp = xReadVlc( vlc_adaptive );
      if ( tmp > atable[vlc_adaptive] )
      {
        vlc_adaptive++;
      }
      if ( tmp )
      {
        xReadFlag( sign );
        if ( sign )
        {
          tmp = -tmp;
        }
      }
      scoeff[i] = tmp;
      i++;
    }
  }

  return;
}

Void TDecCavlc::xParseCoeff8x8(TCoeff* scoeff, int n)
{
  Int i;
  UInt sign;
  Int tmp;
  LastCoeffStruct combo;
  Int vlc,cn,this_pos;
  Int maxrun;
  Int last_position;
  Int atable[5] = {4,6,14,28,0xfffffff};
  Int vlc_adaptive=0;
  Int done;

  static const Int switch_thr[10] = {49,49,0,49,49,0,49,49,49,49};
  Int sum_big_coef = 0;

  for (i = 0; i < 64; i++)
  {
    scoeff[i] = 0;
  }

  /* Get the last nonzero coeff */
  {
    Int x,y,cx,cy,vlcNum;

    /* Decode according to current LP table */
    // ADAPT_VLC_NUM
    vlcNum = g_auiLastPosVlcNum[n][Min(16,m_uiLastPosVlcIndex[n])];
    tmp = xReadVlc( vlcNum );
    cn = m_uiLPTableD8[n][tmp];
    combo.level = (cn>63);
    combo.last_pos = cn&0x3f;

    /* Adapt LP table */
    cx = tmp;
    cy = Max(0,cx-1);
    x = cn;
    y = m_uiLPTableD8[n][cy];
    m_uiLPTableD8[n][cy] = x;
    m_uiLPTableD8[n][cx] = y;
    // ADAPT_VLC_NUM
    m_uiLastPosVlcIndex[n] += cx == m_uiLastPosVlcIndex[n] ? 0 : (cx < m_uiLastPosVlcIndex[n] ? -1 : 1);
  }

  if (combo.level == 1)
  {
    tmp = xReadVlc( 0 );
    sign = tmp&1;
    tmp = (tmp>>1)+2;
  }
  else
  {
    tmp = 1;
    xReadFlag( sign );
  }
  if ( sign )
  {
    tmp = -tmp;
  }

  last_position = combo.last_pos;
  this_pos = 63 - last_position;
  scoeff[this_pos] = tmp;
  i = this_pos;
  i++;

  done = 0;
  {
    while (!done && i < 64)
    {
      maxrun = 63-i;
      if (maxrun > 27)
      {
        maxrun = 28;
        vlc = 3;
      }
      else
      {
        vlc = g_auiVlcTable8x8[maxrun];
      }

      /* Go into run mode */
      cn = xReadVlc( vlc );
      combo = g_acstructLumaRun8x8[maxrun][cn];
      i += combo.last_pos;
      /* No sign for last zeroes */
      if (i < 64)
      {
        if (combo.level == 1)
        {
          tmp = xReadVlc( 0 );
          sign = tmp&1;
          tmp = (tmp>>1)+2;

          sum_big_coef += tmp;
          if (i > switch_thr[n] || sum_big_coef > 2)
          {
            done = 1;
          }
        }
        else
        {
          tmp = 1;
          xReadFlag( sign );
        }
        if ( sign )
        {
          tmp = -tmp;
        }
        scoeff[i] = tmp;
      }
      i++;
    }
  }
  if (i < 64)
  {
    /* Get the rest in level mode */
    while (i<64)
    {
      tmp = xReadVlc( vlc_adaptive );

      if (tmp>atable[vlc_adaptive])
      {
        vlc_adaptive++;
      }
      if (tmp)
      {
        xReadFlag( sign );
        if ( sign )
        {
          tmp = -tmp;
        }
      }
      scoeff[i] = tmp;
      i++;
    }
  }
  return;
}

#ifdef QC_SIFO

Void TDecCavlc::parseSwitched_Filters (TComSlice*& rpcSlice, TComPrediction* m_cPrediction)
{
  UInt  uiCode;
  if(rpcSlice->getSliceType() == I_SLICE)
  {
    m_cPrediction->setPredictFilterP(0);
    m_cPrediction->setPredictFilterB(0);
    for(UInt sub_pos = 0; sub_pos < 16; ++sub_pos)
    {
      m_cPrediction->setPrevP_SIFOFilter(0, sub_pos);
      m_cPrediction->setPrevB_SIFOFilter(0, sub_pos);
    }
  }

  if(rpcSlice->getSliceType() != I_SLICE)
  {
    UInt num_AVALABLE_FILTERS = m_cPrediction->getNum_AvailableFilters();
    UInt num_SIFO = m_cPrediction->getNum_SIFOFilters();

#if SIFO_DIF_COMPATIBILITY==1    //16,17,18,19 ----> 4,5,6,7
    UInt DIF_filter_position = num_SIFO - num_AVALABLE_FILTERS;
    if(rpcSlice->getSPS()->getDIFTap()==6 && rpcSlice->getSliceType()==B_SLICE)
      num_AVALABLE_FILTERS <<= 1;
#endif

    Int bitsPerFilter=(Int)ceil(log10((Double)num_AVALABLE_FILTERS)/log10((Double)2)); 
    Int bitsPer2Filters=(Int)ceil(log10((Double)num_SIFO)/log10((Double)2)); 
    Int sub_pos, bestFilter,predictFilterP,predictFilterB;
    Int SIFO_filter[16],prevFilterP[16],prevFilterB[16];

    //========= get the values of previous frame SIFO filters=====
    if (rpcSlice->getSliceType() == P_SLICE)
    {
      predictFilterP = m_cPrediction->getPredictFilterP();
      for(sub_pos = 1; sub_pos < 16; ++sub_pos)
        prevFilterP[sub_pos] = m_cPrediction->getPrevP_SIFOFilter(sub_pos);
    }
    else
    {
      predictFilterB = m_cPrediction->getPredictFilterB();
      for(sub_pos = 1; sub_pos < 16; ++sub_pos)
        prevFilterB[sub_pos] = m_cPrediction->getPrevB_SIFOFilter(sub_pos);
    }
    //==============================
#ifdef QC_SIFO_PRED
    UInt predict_filter_flag;
    xReadFlag ( predict_filter_flag );
    if (predict_filter_flag && rpcSlice->getSliceType()==P_SLICE && predictFilterP<2)
      predict_filter_flag = 0;
    if (predict_filter_flag && rpcSlice->getSliceType()==B_SLICE && predictFilterB<2)
      predict_filter_flag = 0;
#endif


    if (rpcSlice->getSliceType() == P_SLICE)
    {
#ifdef QC_SIFO_PRED
      if (!predict_filter_flag)
#else
      if (predictFilterP < 2)
#endif
      {
        xReadCode(1,uiCode);
        if(uiCode)    
        {
          for(UInt sub_pos = 1; sub_pos < 16; ++sub_pos)
          {
            if (sub_pos<=4 || sub_pos==8 || sub_pos==12)
              xReadCode(bitsPerFilter, uiCode);
            else
              xReadCode(bitsPer2Filters, uiCode);

            SIFO_filter[sub_pos] = uiCode;
          }
        }
        else
        {
          xReadCode(1,uiCode);
          if(uiCode)
          {
            for(UInt sub_pos = 1; sub_pos < 16; ++sub_pos)
            {
              xReadCode(bitsPerFilter, uiCode);
              SIFO_filter[sub_pos] = uiCode;
            }
          }
          else
          {
            xReadCode(bitsPerFilter, uiCode);
            bestFilter = uiCode;
            //m_cPrediction->setBestFilter(bestFilter);
            for(sub_pos = 1; sub_pos < 16; ++sub_pos)
            {
              SIFO_filter[sub_pos] = bestFilter;
            }
          }
        }
      }
      else
      {
        xReadCode(1,uiCode);
        if(uiCode)
        {
          for(sub_pos = 1; sub_pos < 16; ++sub_pos)
          {
            xReadCode(1,uiCode);
            if(uiCode)
            {
              SIFO_filter[sub_pos] = prevFilterP[sub_pos];
            }
            else
            {
              if (sub_pos<=4 || sub_pos==8 || sub_pos==12)
                xReadCode(bitsPerFilter, uiCode);
              else
                xReadCode(bitsPer2Filters, uiCode);

              SIFO_filter[sub_pos] = uiCode;
            }
          }
        }
        else
        {
          xReadCode(bitsPerFilter, uiCode);
          bestFilter = uiCode;
          //m_cPrediction->setBestFilter(bestFilter);
          for(sub_pos = 1; sub_pos < 16; ++sub_pos)
          {
            SIFO_filter[sub_pos] = bestFilter;
          }
        }
      }
      predictFilterP++;
      if (predictFilterP>2)
        predictFilterP=2;
    }
    else  //B slice
    {
#ifdef QC_SIFO_PRED
      if (!predict_filter_flag)
#else
      if (predictFilterB < 2)
#endif
      {
        xReadCode(1,uiCode);
        if(uiCode)    
        {
          //m_cPrediction->setBestFilter(0);
          for(sub_pos = 1; sub_pos < 16; ++sub_pos)
          {
            xReadCode(bitsPerFilter, uiCode);
            SIFO_filter[sub_pos] = uiCode;
#if SIFO_DIF_COMPATIBILITY==1    //4,5,6,7 ----> 16,17,18,19
            if(SIFO_filter[sub_pos] >= m_cPrediction->getNum_AvailableFilters() && rpcSlice->getSPS()->getDIFTap()==6)
              SIFO_filter[sub_pos] += (num_SIFO-num_AVALABLE_FILTERS);
#endif
          }
        }
        else
        {
          xReadCode(bitsPerFilter, uiCode);
          bestFilter = uiCode;
#if SIFO_DIF_COMPATIBILITY==1    //4,5,6,7 ----> 16,17,18,19
          if(bestFilter >= m_cPrediction->getNum_AvailableFilters() && rpcSlice->getSPS()->getDIFTap()==6)
            bestFilter += (num_SIFO-num_AVALABLE_FILTERS);
#endif
          //m_cPrediction->setBestFilter(bestFilter);
          for(sub_pos = 1; sub_pos < 16; ++sub_pos)
          {
            SIFO_filter[sub_pos] = bestFilter;
          }
        }
      }
      else
      {
        xReadCode(1,uiCode);
        if(uiCode)    //bestFilter=0
        {
          //m_cPrediction->setBestFilter(0);
          for(sub_pos = 1; sub_pos < 16; ++sub_pos)
          {
            xReadCode(1, uiCode);
            if(uiCode==0)
            {
              xReadCode(bitsPerFilter, uiCode);
              SIFO_filter[sub_pos] = uiCode;
#if SIFO_DIF_COMPATIBILITY==1    //4,5,6,7 ----> 16,17,18,19
            if(SIFO_filter[sub_pos] >= m_cPrediction->getNum_AvailableFilters() && rpcSlice->getSPS()->getDIFTap()==6)
              SIFO_filter[sub_pos] += (num_SIFO-num_AVALABLE_FILTERS);
#endif
            }
            else
            {
              SIFO_filter[sub_pos] = prevFilterB[sub_pos];
            }
          }
        }
        else
        {
          xReadCode(bitsPerFilter, uiCode);
          bestFilter = uiCode;
#if SIFO_DIF_COMPATIBILITY==1    //4,5,6,7 ----> 16,17,18,19
          if(bestFilter >= m_cPrediction->getNum_AvailableFilters() && rpcSlice->getSPS()->getDIFTap()==6)
            bestFilter += (num_SIFO-num_AVALABLE_FILTERS);
#endif
          //m_cPrediction->setBestFilter(bestFilter);
          for(sub_pos = 1; sub_pos < 16; ++sub_pos)
          {
            SIFO_filter[sub_pos] = bestFilter;
          }
        }
      }
      predictFilterB++;
      if (predictFilterB>2)
        predictFilterB=2;
    }

    //========= set the values =====
    if (rpcSlice->getSliceType() == P_SLICE)
    {
      m_cPrediction->setPredictFilterP(predictFilterP);
      for(sub_pos = 1; sub_pos < 16; ++sub_pos)
      {
        m_cPrediction->setSIFOFilter      (SIFO_filter[sub_pos], sub_pos);
        m_cPrediction->setPrevP_SIFOFilter(SIFO_filter[sub_pos], sub_pos);
      }
    }
    else
    {
      m_cPrediction->setPredictFilterB(predictFilterB);
      for(sub_pos = 1; sub_pos < 16; ++sub_pos)
      {
        m_cPrediction->setSIFOFilter      (SIFO_filter[sub_pos], sub_pos);
        m_cPrediction->setPrevB_SIFOFilter(SIFO_filter[sub_pos], sub_pos);
      }
    }
    //==============================

  }

#ifdef QC_SIFO
  if(rpcSlice->getSliceType() != I_SLICE)
  {
    Int listNo = (rpcSlice->getSliceType() == B_SLICE)? 2: 1;
    Int iCode;
    UInt uiCode;

    m_cPrediction->setOffsets_toZero();

    for(Int list = 0; list < listNo; ++list) 
    {
      xReadCode(1,uiCode);
      UInt nonzero = uiCode;
      if(nonzero)
      {
        for(UInt frame = 0; frame < rpcSlice->getNumRefIdx(RefPicList(list)); ++frame)
        {
          if(frame == 0)     
          {    
            for(UInt sub_pos = 0; sub_pos < 16; ++sub_pos)   
            {
              xReadSvlc(iCode);
              iCode *= (1<<g_uiBitIncrement);
              m_cPrediction->setSubpelOffset(iCode,list,sub_pos);
            }         
          }
          else              
          {
            xReadSvlc(iCode);
            iCode *= (1<<g_uiBitIncrement);
            m_cPrediction->setFrameOffset(iCode,list,frame);
          }
        }
      }
    }
  }
#endif

}
#endif
>>>>>>> upstream/master
