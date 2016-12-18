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

/** \file     ContextTables.h
    \brief    Defines constants and tables for SBAC
    \todo     number of context models is not matched to actual use, should be fixed
*/

#ifndef __CONTEXTTABLES__
#define __CONTEXTTABLES__

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define MAX_NUM_CTX_MOD             512       ///< maximum number of supported contexts

#define NUM_SPLIT_FLAG_CTX            3       ///< number of context models for split flag
#define NUM_SKIP_FLAG_CTX             3       ///< number of context models for skip flag

#define NUM_MERGE_FLAG_EXT_CTX        1       ///< number of context models for merge flag of merge extended
#define NUM_MERGE_IDX_EXT_CTX         1       ///< number of context models for merge index of merge extended

#define NUM_PART_SIZE_CTX             4       ///< number of context models for partition size
#define NUM_PRED_MODE_CTX             1       ///< number of context models for prediction mode

#define NUM_INTRA_PREDICT_CTX         1       ///< number of context models for intra prediction

#define NUM_CHROMA_PRED_CTX           2       ///< number of context models for intra prediction (chroma)
#define NUM_INTER_DIR_CTX             5       ///< number of context models for inter prediction direction
#define NUM_MV_RES_CTX                2       ///< number of context models for motion vector difference
#define NUM_CHROMA_QP_ADJ_FLAG_CTX    1       ///< number of context models for chroma_qp_adjustment_flag
#define NUM_CHROMA_QP_ADJ_IDC_CTX     1       ///< number of context models for chroma_qp_adjustment_idc

#define NUM_REF_NO_CTX                2       ///< number of context models for reference index
#define NUM_TRANS_SUBDIV_FLAG_CTX     3       ///< number of context models for transform subdivision flags
#define NUM_QT_ROOT_CBF_CTX           1       ///< number of context models for QT ROOT CBF
#define NUM_DELTA_QP_CTX              3       ///< number of context models for dQP

#define NUM_SIG_CG_FLAG_CTX           2       ///< number of context models for MULTI_LEVEL_SIGNIFICANCE
#define NUM_EXPLICIT_RDPCM_FLAG_CTX   1       ///< number of context models for the flag which specifies whether to use RDPCM on inter coded residues
#define NUM_EXPLICIT_RDPCM_DIR_CTX    1       ///< number of context models for the flag which specifies which RDPCM direction is used on inter coded residues

//--------------------------------------------------------------------------------------------------

// context size definitions for significance map

#define NUM_SIG_FLAG_CTX_LUMA        28       ///< number of context models for luma sig flag
#define NUM_SIG_FLAG_CTX_CHROMA      16       ///< number of context models for chroma sig flag

//                                                                                                           |----Luma-----|  |---Chroma----|
static const UInt significanceMapContextSetStart         [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {0,  9, 21, 27}, {0,  9, 12, 15} };
static const UInt significanceMapContextSetSize          [MAX_NUM_CHANNEL_TYPE][CONTEXT_NUMBER_OF_TYPES] = { {9, 12,  6,  1}, {9,  3,  3,  1} };
static const UInt nonDiagonalScan8x8ContextOffset        [MAX_NUM_CHANNEL_TYPE]                          = {  6,               0              };
static const UInt notFirstGroupNeighbourhoodContextOffset[MAX_NUM_CHANNEL_TYPE]                          = {  3,               0              };

//------------------

#define NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x4  3
#define NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x4  1

//------------------

#define FIRST_SIG_FLAG_CTX_LUMA                   0
#define FIRST_SIG_FLAG_CTX_CHROMA     (FIRST_SIG_FLAG_CTX_LUMA + NUM_SIG_FLAG_CTX_LUMA)

#define NUM_SIG_FLAG_CTX              (NUM_SIG_FLAG_CTX_LUMA + NUM_SIG_FLAG_CTX_CHROMA)       ///< number of context models for sig flag

//--------------------------------------------------------------------------------------------------

// context size definitions for last significant coefficient position

#define NUM_CTX_LAST_FLAG_SETS         2

#define NUM_CTX_LAST_FLAG_XY          15      ///< number of context models for last coefficient position

//--------------------------------------------------------------------------------------------------

// context size definitions for greater-than-one and greater-than-two maps

#define NUM_ONE_FLAG_CTX_PER_SET       4      ///< number of context models for greater than 1 flag in a set
#define NUM_ABS_FLAG_CTX_PER_SET       1      ///< number of context models for greater than 2 flag in a set

//------------------

#define NUM_CTX_SETS_LUMA              4      ///< number of context model sets for luminance
#define NUM_CTX_SETS_CHROMA            2      ///< number of context model sets for combined chrominance

#define FIRST_CTX_SET_LUMA             0      ///< index of first luminance context set

//------------------

#define NUM_ONE_FLAG_CTX_LUMA         (NUM_ONE_FLAG_CTX_PER_SET * NUM_CTX_SETS_LUMA)           ///< number of context models for greater than 1 flag of luma
#define NUM_ONE_FLAG_CTX_CHROMA       (NUM_ONE_FLAG_CTX_PER_SET * NUM_CTX_SETS_CHROMA)         ///< number of context models for greater than 1 flag of chroma

#define NUM_ABS_FLAG_CTX_LUMA         (NUM_ABS_FLAG_CTX_PER_SET * NUM_CTX_SETS_LUMA)           ///< number of context models for greater than 2 flag of luma
#define NUM_ABS_FLAG_CTX_CHROMA       (NUM_ABS_FLAG_CTX_PER_SET * NUM_CTX_SETS_CHROMA)         ///< number of context models for greater than 2 flag of chroma

#define NUM_ONE_FLAG_CTX              (NUM_ONE_FLAG_CTX_LUMA + NUM_ONE_FLAG_CTX_CHROMA)        ///< number of context models for greater than 1 flag
#define NUM_ABS_FLAG_CTX              (NUM_ABS_FLAG_CTX_LUMA + NUM_ABS_FLAG_CTX_CHROMA)        ///< number of context models for greater than 2 flag

#define FIRST_CTX_SET_CHROMA          (FIRST_CTX_SET_LUMA + NUM_CTX_SETS_LUMA)                 ///< index of first chrominance context set

//--------------------------------------------------------------------------------------------------

// context size definitions for CBF

#define NUM_QT_CBF_CTX_SETS           2

#define NUM_QT_CBF_CTX_PER_SET        5       ///< number of context models for QT CBF

#define FIRST_CBF_CTX_LUMA            0       ///< index of first luminance CBF context

#define FIRST_CBF_CTX_CHROMA          (FIRST_CBF_CTX_LUMA + NUM_QT_CBF_CTX_PER_SET)  ///< index of first chrominance CBF context


//--------------------------------------------------------------------------------------------------

#define NUM_MVP_IDX_CTX               1       ///< number of context models for MVP index

#define NUM_SAO_MERGE_FLAG_CTX        1       ///< number of context models for SAO merge flags
#define NUM_SAO_TYPE_IDX_CTX          1       ///< number of context models for SAO type index

#define NUM_TRANSFORMSKIP_FLAG_CTX    1       ///< number of context models for transform skipping

#define NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX  1

#define NUM_CROSS_COMPONENT_PREDICTION_CTX 10

#define CNU                          154      ///< dummy initialization value for unused context models 'Context model Not Used'


// ====================================================================================================================
// Tables
// ====================================================================================================================

// initial probability for cu_transquant_bypass flag
static const UChar
INIT_CU_TRANSQUANT_BYPASS_FLAG[NUMBER_OF_SLICE_TYPES][NUM_CU_TRANSQUANT_BYPASS_FLAG_CTX] =
{
  { 154 },
  { 154 },
  { 154 },
};

// initial probability for split flag
static const UChar
INIT_SPLIT_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SPLIT_FLAG_CTX] =
{
  { 107,  139,  126, },
  { 107,  139,  126, },
  { 139,  141,  157, },
};

static const UChar
INIT_SKIP_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SKIP_FLAG_CTX] =
{
  { 197,  185,  201, },
  { 197,  185,  201, },
  { CNU,  CNU,  CNU, },
};

static const UChar
INIT_MERGE_FLAG_EXT[NUMBER_OF_SLICE_TYPES][NUM_MERGE_FLAG_EXT_CTX] =
{
  { 154, },
  { 110, },
  { CNU, },
};

static const UChar
INIT_MERGE_IDX_EXT[NUMBER_OF_SLICE_TYPES][NUM_MERGE_IDX_EXT_CTX] =
{
  { 137, },
  { 122, },
  { CNU, },
};

static const UChar
INIT_PART_SIZE[NUMBER_OF_SLICE_TYPES][NUM_PART_SIZE_CTX] =
{
  { 154,  139,  154, 154 },
  { 154,  139,  154, 154 },
  { 184,  CNU,  CNU, CNU },
};

static const UChar
INIT_PRED_MODE[NUMBER_OF_SLICE_TYPES][NUM_PRED_MODE_CTX] =
{
  { 134, },
  { 149, },
  { CNU, },
};

static const UChar
INIT_INTRA_PRED_MODE[NUMBER_OF_SLICE_TYPES][NUM_INTRA_PREDICT_CTX] =
{
  { 183, },
  { 154, },
  { 184, },
};

static const UChar
INIT_CHROMA_PRED_MODE[NUMBER_OF_SLICE_TYPES][NUM_CHROMA_PRED_CTX] =
{
  { 152,  139, },
  { 152,  139, },
  {  63,  139, },
};

static const UChar
INIT_INTER_DIR[NUMBER_OF_SLICE_TYPES][NUM_INTER_DIR_CTX] =
{
  {  95,   79,   63,   31,  31, },
  {  95,   79,   63,   31,  31, },
  { CNU,  CNU,  CNU,  CNU, CNU, },
};

static const UChar
INIT_MVD[NUMBER_OF_SLICE_TYPES][NUM_MV_RES_CTX] =
{
  { 169,  198, },
  { 140,  198, },
  { CNU,  CNU, },
};

static const UChar
INIT_REF_PIC[NUMBER_OF_SLICE_TYPES][NUM_REF_NO_CTX] =
{
  { 153,  153 },
  { 153,  153 },
  { CNU,  CNU },
};

static const UChar
INIT_DQP[NUMBER_OF_SLICE_TYPES][NUM_DELTA_QP_CTX] =
{
  { 154,  154,  154, },
  { 154,  154,  154, },
  { 154,  154,  154, },
};

static const UChar
INIT_CHROMA_QP_ADJ_FLAG[NUMBER_OF_SLICE_TYPES][NUM_CHROMA_QP_ADJ_FLAG_CTX] =
{
  { 154, },
  { 154, },
  { 154, },
};

static const UChar
INIT_CHROMA_QP_ADJ_IDC[NUMBER_OF_SLICE_TYPES][NUM_CHROMA_QP_ADJ_IDC_CTX] =
{
  { 154, },
  { 154, },
  { 154, },
};

//--------------------------------------------------------------------------------------------------

//Initialisation for CBF

//                                 |---------Luminance---------|
#define BSLICE_LUMA_CBF_CONTEXT     153,  111,  CNU,  CNU,  CNU
#define PSLICE_LUMA_CBF_CONTEXT     153,  111,  CNU,  CNU,  CNU
#define ISLICE_LUMA_CBF_CONTEXT     111,  141,  CNU,  CNU,  CNU
//                                 |--------Chrominance--------|
#define BSLICE_CHROMA_CBF_CONTEXT   149,   92,  167,  154,  154
#define PSLICE_CHROMA_CBF_CONTEXT   149,  107,  167,  154,  154
#define ISLICE_CHROMA_CBF_CONTEXT    94,  138,  182,  154,  154


static const UChar
INIT_QT_CBF[NUMBER_OF_SLICE_TYPES][NUM_QT_CBF_CTX_SETS * NUM_QT_CBF_CTX_PER_SET] =
{
  { BSLICE_LUMA_CBF_CONTEXT, BSLICE_CHROMA_CBF_CONTEXT },
  { PSLICE_LUMA_CBF_CONTEXT, PSLICE_CHROMA_CBF_CONTEXT },
  { ISLICE_LUMA_CBF_CONTEXT, ISLICE_CHROMA_CBF_CONTEXT },
};


//--------------------------------------------------------------------------------------------------

static const UChar
INIT_QT_ROOT_CBF[NUMBER_OF_SLICE_TYPES][NUM_QT_ROOT_CBF_CTX] =
{
  {  79, },
  {  79, },
  { CNU, },
};


//--------------------------------------------------------------------------------------------------

//Initialisation for last-significant-position

//                                           |------------------------------Luminance----------------------------------|
#define BSLICE_LUMA_LAST_POSITION_CONTEXT     125, 110, 124, 110,  95,  94, 125, 111, 111,  79, 125, 126, 111, 111,  79
#define PSLICE_LUMA_LAST_POSITION_CONTEXT     125, 110,  94, 110,  95,  79, 125, 111, 110,  78, 110, 111, 111,  95,  94
#define ISLICE_LUMA_LAST_POSITION_CONTEXT     110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111,  79
//                                           |------------------------------Chrominance--------------------------------|
#define BSLICE_CHROMA_LAST_POSITION_CONTEXT   108, 123,  93, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU
#define PSLICE_CHROMA_LAST_POSITION_CONTEXT   108, 123, 108, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU
#define ISLICE_CHROMA_LAST_POSITION_CONTEXT   108, 123,  63, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU


static const UChar
INIT_LAST[NUMBER_OF_SLICE_TYPES][NUM_CTX_LAST_FLAG_SETS * NUM_CTX_LAST_FLAG_XY] =
{
  { BSLICE_LUMA_LAST_POSITION_CONTEXT, BSLICE_CHROMA_LAST_POSITION_CONTEXT },
  { PSLICE_LUMA_LAST_POSITION_CONTEXT, PSLICE_CHROMA_LAST_POSITION_CONTEXT },
  { ISLICE_LUMA_LAST_POSITION_CONTEXT, ISLICE_CHROMA_LAST_POSITION_CONTEXT },
};


//--------------------------------------------------------------------------------------------------

static const UChar
INIT_SIG_CG_FLAG[NUMBER_OF_SLICE_TYPES][2 * NUM_SIG_CG_FLAG_CTX] =
{
  { 121,  140,
    61,  154,
  },
  { 121,  140,
    61,  154,
  },
  {  91,  171,
    134,  141,
  },
};


//--------------------------------------------------------------------------------------------------

//Initialisation for significance map

//                                          |-DC-|  |-----------------4x4------------------|  |------8x8 Diagonal Scan------|  |----8x8 Non-Diagonal Scan----|  |-NxN First group-|  |-NxN Other group-| |-Single context-|
//                                          |    |  |                                      |  |-First Group-| |-Other Group-|  |-First Group-| |-Other Group-|  |                 |  |                 | |                |
#define BSLICE_LUMA_SIGNIFICANCE_CONTEXT     170,    154, 139, 153, 139, 123, 123,  63, 124,   166, 183, 140,  136, 153, 154,   166, 183, 140,  136, 153, 154,   166,   183,   140,   136,   153,   154,        140
#define PSLICE_LUMA_SIGNIFICANCE_CONTEXT     155,    154, 139, 153, 139, 123, 123,  63, 153,   166, 183, 140,  136, 153, 154,   166, 183, 140,  136, 153, 154,   166,   183,   140,   136,   153,   154,        140
#define ISLICE_LUMA_SIGNIFICANCE_CONTEXT     111,    111, 125, 110, 110,  94, 124, 108, 124,   107, 125, 141,  179, 153, 125,   107, 125, 141,  179, 153, 125,   107,   125,   141,   179,   153,   125,        141

//                                          |-DC-|  |-----------------4x4------------------|  |-8x8 Any group-|  |-NxN Any group-| |-Single context-|
#define BSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 138, 138, 122, 121, 122, 121, 167,   151,  183,  140,   151,  183,  140,        140
#define PSLICE_CHROMA_SIGNIFICANCE_CONTEXT   170,    153, 123, 123, 107, 121, 107, 121, 167,   151,  183,  140,   151,  183,  140,        140
#define ISLICE_CHROMA_SIGNIFICANCE_CONTEXT   140,    139, 182, 182, 152, 136, 152, 136, 153,   136,  139,  111,   136,  139,  111,        111

//------------------------------------------------

static const UChar
INIT_SIG_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SIG_FLAG_CTX] =
{
  { BSLICE_LUMA_SIGNIFICANCE_CONTEXT, BSLICE_CHROMA_SIGNIFICANCE_CONTEXT },
  { PSLICE_LUMA_SIGNIFICANCE_CONTEXT, PSLICE_CHROMA_SIGNIFICANCE_CONTEXT },
  { ISLICE_LUMA_SIGNIFICANCE_CONTEXT, ISLICE_CHROMA_SIGNIFICANCE_CONTEXT },
};


//--------------------------------------------------------------------------------------------------

//Initialisation for greater-than-one flags and greater-than-two flags

//                                 |------Set 0-------| |------Set 1-------| |------Set 2-------| |------Set 3-------|
#define BSLICE_LUMA_ONE_CONTEXT     154, 196, 167, 167,  154, 152, 167, 182,  182, 134, 149, 136,  153, 121, 136, 122
#define PSLICE_LUMA_ONE_CONTEXT     154, 196, 196, 167,  154, 152, 167, 182,  182, 134, 149, 136,  153, 121, 136, 137
#define ISLICE_LUMA_ONE_CONTEXT     140,  92, 137, 138,  140, 152, 138, 139,  153,  74, 149,  92,  139, 107, 122, 152

#define BSLICE_LUMA_ABS_CONTEXT     107,                 167,                  91,                 107
#define PSLICE_LUMA_ABS_CONTEXT     107,                 167,                  91,                 122
#define ISLICE_LUMA_ABS_CONTEXT     138,                 153,                 136,                 167

//                                 |------Set 4-------| |------Set 5-------|
#define BSLICE_CHROMA_ONE_CONTEXT   169, 208, 166, 167,  154, 152, 167, 182
#define PSLICE_CHROMA_ONE_CONTEXT   169, 194, 166, 167,  154, 167, 137, 182
#define ISLICE_CHROMA_ONE_CONTEXT   140, 179, 166, 182,  140, 227, 122, 197

#define BSLICE_CHROMA_ABS_CONTEXT   107,                 167
#define PSLICE_CHROMA_ABS_CONTEXT   107,                 167
#define ISLICE_CHROMA_ABS_CONTEXT   152,                 152


//------------------------------------------------

static const UChar
INIT_ONE_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ONE_FLAG_CTX] =
{
  { BSLICE_LUMA_ONE_CONTEXT, BSLICE_CHROMA_ONE_CONTEXT },
  { PSLICE_LUMA_ONE_CONTEXT, PSLICE_CHROMA_ONE_CONTEXT },
  { ISLICE_LUMA_ONE_CONTEXT, ISLICE_CHROMA_ONE_CONTEXT },
};

static const UChar
INIT_ABS_FLAG[NUMBER_OF_SLICE_TYPES][NUM_ABS_FLAG_CTX] =
{
  { BSLICE_LUMA_ABS_CONTEXT, BSLICE_CHROMA_ABS_CONTEXT },
  { PSLICE_LUMA_ABS_CONTEXT, PSLICE_CHROMA_ABS_CONTEXT },
  { ISLICE_LUMA_ABS_CONTEXT, ISLICE_CHROMA_ABS_CONTEXT },
};


//--------------------------------------------------------------------------------------------------

static const UChar
INIT_MVP_IDX[NUMBER_OF_SLICE_TYPES][NUM_MVP_IDX_CTX] =
{
  { 168, },
  { 168, },
  { CNU, },
};

static const UChar
INIT_SAO_MERGE_FLAG[NUMBER_OF_SLICE_TYPES][NUM_SAO_MERGE_FLAG_CTX] =
{
  { 153,  },
  { 153,  },
  { 153,  },
};

static const UChar
INIT_SAO_TYPE_IDX[NUMBER_OF_SLICE_TYPES][NUM_SAO_TYPE_IDX_CTX] =
{
  { 160, },
  { 185, },
  { 200, },
};

static const UChar
INIT_TRANS_SUBDIV_FLAG[NUMBER_OF_SLICE_TYPES][NUM_TRANS_SUBDIV_FLAG_CTX] =
{
  { 224,  167,  122, },
  { 124,  138,   94, },
  { 153,  138,  138, },
};

static const UChar
INIT_TRANSFORMSKIP_FLAG[NUMBER_OF_SLICE_TYPES][2*NUM_TRANSFORMSKIP_FLAG_CTX] =
{
  { 139,  139},
  { 139,  139},
  { 139,  139},
};

static const UChar
INIT_EXPLICIT_RDPCM_FLAG[NUMBER_OF_SLICE_TYPES][2*NUM_EXPLICIT_RDPCM_FLAG_CTX] =
{
  {139, 139},
  {139, 139},
  {CNU, CNU}
};

static const UChar
INIT_EXPLICIT_RDPCM_DIR[NUMBER_OF_SLICE_TYPES][2*NUM_EXPLICIT_RDPCM_DIR_CTX] =
{
  {139, 139},
  {139, 139},
  {CNU, CNU}
};

static const UChar
INIT_CROSS_COMPONENT_PREDICTION[NUMBER_OF_SLICE_TYPES][NUM_CROSS_COMPONENT_PREDICTION_CTX] =
{
  { 154, 154, 154, 154, 154, 154, 154, 154, 154, 154 },
  { 154, 154, 154, 154, 154, 154, 154, 154, 154, 154 },
  { 154, 154, 154, 154, 154, 154, 154, 154, 154, 154 },
};

//! \}

#endif
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

/** \file     ContextTables.h
    \brief    Defines constants and tables for SBAC
    \todo     number of context models is not matched to actual use, should be fixed
*/

#ifndef __CONTEXTTABLES__
#define __CONTEXTTABLES__

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define NUM_SPLIT_FLAG_CTX            3       ///< number of context models for split flag
#define NUM_SKIP_FLAG_CTX             3       ///< number of context models for skip flag

#if HHI_MRG
#define NUM_MERGE_FLAG_CTX            3       ///< number of context models for merge flag
#define NUM_MERGE_INDEX_CTX           3       ///< number of context models for merge index
#endif

#define NUM_ALF_CTRL_FLAG_CTX         3       ///< number of context models for ALF control flag
#define NUM_PART_SIZE_CTX             5       ///< number of context models for partition size
#define NUM_CU_X_POS_CTX              2       ///< number of context models for partition size (AMP)
#define NUM_CU_Y_POS_CTX              2       ///< number of context models for partition size (AMP)
#define NUM_PRED_MODE_CTX             2       ///< number of context models for prediction mode
#define NUM_ADI_CTX                   2       ///< number of context models for intra prediction

#if HHI_AIS
#define NUM_ADI_FILT_CTX              5       ///< BB: number of context models for AIS flag (one for every row in Table 2‑5 in JCTVC-A125r1 except DC)
#endif

#define NUM_CHROMA_PRED_CTX           4       ///< number of context models for intra prediction (chroma)
#define NUM_INTER_DIR_CTX             4       ///< number of context models for inter prediction direction
#define NUM_MV_RES_CTX                7       ///< number of context models for motion vector difference

#ifdef DCM_PBIC
#define NUM_IC_RES_CTX                7       ///< number of context models for IC parameter difference
#endif

#define NUM_REF_NO_CTX                6       ///< number of context models for reference index
#if HHI_RQT
#define NUM_TRANS_SUBDIV_FLAG_CTX     10      ///< number of context models for transform subdivision flags
#define NUM_QT_CBF_CTX                15      ///< number of context models for QT CBF
#if HHI_RQT_ROOT
#define NUM_QT_ROOT_CBF_CTX           4       ///< number of context models for QT ROOT CBF
#endif
#endif
#define NUM_TRANS_IDX_CTX             4       ///< number of context models for transform index
#define NUM_DELTA_QP_CTX              4       ///< number of context models for dQP
#define NUM_CBF_CTX                   4       ///< number of context models for CBF

#if HHI_TRANSFORM_CODING
#define NUM_SIG_FLAG_CTX              16      ///< number of context models for sig flag
#define NUM_LAST_FLAG_CTX             16      ///< number of context models for last flag
#define NUM_ABS_GREATER_ONE_CTX       40      ///< number of context models for greater than one
#define NUM_COEFF_LEVEL_MINUS_ONE_CTX 40      ///< number of context models for magnitude
#else
#define NUM_MAP_CTX										15			///< number of context models for sigmap
#define NUM_LAST_CTX									15			///< number of context models for lastbit
#define NUM_ONE_CTX										5				///< number of context models for greater than one
#define NUM_ABS_CTX										5				///< number of context models for magnitude
#endif

#define NUM_MVP_IDX_CTX               2       ///< number of context models for MVP index

#ifdef DCM_PBIC
#define NUM_ICP_IDX_CTX               2       ///< number of context models for ICP index
#define NUM_ZTREE_MV0_CTX             1       ///< number of context models for coding zero flag related to MVD & ICD
#define NUM_ZTREE_MV1_CTX             6       ///< number of context models for coding MVD & ICD (Uni-pred case)
#define NUM_ZTREE_MV2_CTX            12       ///< number of context models for coding MVD & ICD ( Bi-pred case)
#endif

#define NUM_ROT_IDX_CTX               3       ///< number of context models for ROT index
#define NUM_CIP_FLAG_CTX              3       ///< number of context models for CIP flag
#define NUM_ALF_FLAG_CTX              1       ///< number of context models for ALF flag
#define NUM_ALF_UVLC_CTX              2       ///< number of context models for ALF UVLC (filter length)
#define NUM_ALF_SVLC_CTX              3       ///< number of context models for ALF SVLC (filter coeff.)

#if PLANAR_INTRA
#define NUM_PLANAR_INTRA_CTX          2
#endif

#if HHI_ALF
#define NUM_ALF_SPLITFLAG_CTX         1       ///< number of context models for ALF split flag
#endif
#ifdef QC_AMVRES
#define NUM_MV_RES_FALG_CTX			  3			  ///< number of context models for motion vector resolution flag
#endif
// ====================================================================================================================
// Tables
// ====================================================================================================================

// mapping table for mapping 8x8 sigmap to 4x4 sigmap
static const int  pos2ctx_map8x8  []  = { 0,  1,  2,  3,  4,  5,  5,  4,  4,  3,  3,  4,  4,  4,  5,  5,
                                          4,  4,  4,  4,  3,  3,  6,  7,  7,  7,  8,  9, 10,  9,  8,  7,
                                          7,  6, 11, 12, 13, 11,  6,  7,  8,  9, 14, 10,  9,  8,  6, 11,
                                         12, 13, 11,  6,  9, 14, 10,  9, 11, 12, 13, 11 ,14, 10, 12, 14};

// mapping table for mapping 8x8 lastbit to 4x4 lastbit
static const int  pos2ctx_last8x8 []  = { 0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
                                          2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
                                          3,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  4,  4,  4,
                                          5,  5,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8};

// no-mapping case
static const int  pos2ctx_nomap   []  = { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15};


// initial probability for split flag
static const Short
INIT_SPLIT_FLAG[3][NUM_SPLIT_FLAG_CTX][2] =
{
  {
    {   -7,   68 }, {  -10,   87 }, {  -10,  105 }
  },
  {
    {  -14,   71 }, {   -6,   73 }, {   -6,   91 }
  },
  {
    {  -14,   71 }, {   -7,   74 }, {  -10,   92 }
  }
};

// initial probability for skip flag
static const Short
INIT_SKIP_FLAG[3][NUM_SKIP_FLAG_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }
  }
};

#if HHI_MRG
// initial probability for merge flag
static const Short
INIT_MERGE_FLAG[3][NUM_MERGE_FLAG_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {   13,   29 }, {    9,   43 }, {   11,   53 }
  },
  {
    {   14,   28 }, {   10,   46 }, {   10,   58 }
  }
};

// initial probability for merge index
static const Short
INIT_MERGE_INDEX[3][NUM_MERGE_INDEX_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {   -1,   63 }, {    0,   64 }, {    0,   64 }
  },
  {
    {   -3,   65 }, {    0,   64 }, {    0,   64 }
  }
};
#endif

// initial probability for PU size
static const Short
INIT_PART_SIZE[3][NUM_PART_SIZE_CTX][2] =
{
  {
    {    0,   73 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }
  },
  {
    {   -1,   64 }, {   -3,   63 }, {    6,   78 }, {    0,   64 }, 
    {    0,   64 }
  },
  {
    {    6,   50 }, {   -1,   56 }, {   13,   53 }, {  -11,   76 }, 
    {  -11,   70 }
  }
};

// initial probability for AMP split position (X)
static const Short
INIT_CU_X_POS[3][NUM_CU_X_POS_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }
  },
  {
    {   -1,   59 }, {    0,   63 }
  },
  {
    {   -1,   55 }, {   -3,   67 }
  }
};

// initial probability for AMP split position (Y)
static const Short
INIT_CU_Y_POS[3][NUM_CU_Y_POS_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   57 }, {   -2,   66 }
  },
  {
    {   -3,   61 }, {   -3,   66 }
  }
};

// initial probability for prediction mode
static const Short
INIT_PRED_MODE[3][NUM_PRED_MODE_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {  -25,   89 }
  },
  {
    {    0,   64 }, {    0,   64 }
  }
};

// initial probability for intra direction of luma
static const Short
INIT_INTRA_PRED_MODE[3][NUM_ADI_CTX][2] =
{
  {
    {    2,   54 }, {   -3,   65 }
  },
  {
    {    0,   50 }, {   -2,   61 }
  },
  {
    {    0,   51 }, {    1,   55 }
  }
};

#if PLANAR_INTRA
static const Short
INIT_PLANAR_INTRA[3][NUM_PLANAR_INTRA_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }
  }
};
#endif

#if HHI_AIS
// BB: initial probability for intra reference pixel filtering
static const Short
INIT_INTRA_PRED_FILT[3][NUM_ADI_FILT_CTX][2] =
{
  {
    {   -2,   62 }, {    0,   59 }, {    8,   44 }, {    4,   76 }, 
    {   56,  -54 }
  },
  {
    {  -14,   83 }, {  -14,   83 }, {  -21,   96 }, {  -11,   96 }, 
    {  -27,   61 }
  },
  {
    {  -19,   94 }, {   -3,   62 }, {  -24,   97 }, {   -7,   82 }, 
    {    0,   64 }
  }
};
#endif

// initial probability for intra direction of chroma
static const Short
INIT_CHROMA_PRED_MODE[3][4][2] =
{
  {
    {  -14,   78 }, {   -5,   78 }, {  -13,  109 }, {  -13,  100 }
  },
  {
    {  -26,  113 }, {  -13,   92 }, {  -23,  110 }, {  -11,  105 }
  },
  {
    {  -20,  101 }, {  -25,  111 }, {  -29,  118 }, {  -24,  121 }
  }
};

// initial probability for temporal direction
static const Short
INIT_INTER_DIR[3][4][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {   -2,   58 }, {   -5,   70 }, {   -9,   85 }, {    1,   61 }
  }
};

// initial probability for motion vector difference
static const Short INIT_MVD[3][14][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {   -6,   80 }, {   -6,   84 }, {   -9,   90 }, {    4,   62 }, 
    {   13,   55 }, {    2,   70 }, {    8,   74 }, {   -6,   77 }, 
    {   -7,   84 }, {   -9,   89 }, {    5,   59 }, {   10,   62 }, 
    {    4,   68 }, {    7,   75 }
  },
  {
    {   -4,   75 }, {   -5,   82 }, {  -12,   94 }, {    7,   55 }, 
    {   11,   59 }, {    6,   63 }, {    8,   71 }, {   -2,   71 }, 
    {   -5,   81 }, {  -21,  111 }, {    6,   58 }, {   10,   60 }, 
    {    5,   64 }, {   10,   67 }
  }
};

#ifdef DCM_PBIC
// initial probability for IC parameter difference
static const Short INIT_ICD[3][21][2] = 
{
  // for I slice
  {{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},
   {0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},
   {0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64}},
  // for P slice
  {{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},
   {0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},
   {0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64}},
  // for B slice
  {{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},
   {0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},
   {0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64}}
};
#endif 

// initial probability for reference frame index
static const Short
INIT_REF_PIC[3][NUM_REF_NO_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {   -6,   59 }, {  -10,   75 }, {   -8,   75 }, {  -17,   96 }, 
    {    1,   59 }, {    0,   64 }
  },
  {
    {   -9,   55 }, {   -9,   71 }, {   -9,   76 }, {  -12,   86 }, 
    {  -18,   55 }, {    0,   64 }
  }
};

#ifdef QC_AMVRES
// initial probability for MV resolution flag
static const Short
INIT_MVRES_FLAG[3][3][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }
  }
};
#endif

// initial probability for dQP
static const Short
INIT_DQP[3][NUM_DELTA_QP_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }
  }
};

// initial probability for CBF
static const Short
INIT_CBF[3][8][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  }
};

#if HHI_RQT
static const Short
INIT_QT_CBF[3][3*NUM_QT_CBF_CTX][2] =
{
  {
    {  -22,  116 }, {   -5,   75 }, {  -16,  112 }, {  -16,  111 }, 
    {  -32,  165 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {  -35,  116 }, 
    {  -12,   61 }, {   -9,   73 }, {  -10,   75 }, {  -14,   96 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {  -29,  104 }, {  -12,   59 }, 
    {   -5,   65 }, {   -6,   67 }, {  -11,   90 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }
  },
  {
    {  -18,   98 }, {  -41,  120 }, {  -29,  117 }, {  -23,  108 }, 
    {  -35,  143 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {  -46,  114 }, 
    {  -42,  119 }, {  -11,   74 }, {  -19,   90 }, {  -42,  139 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {  -43,  107 }, {  -41,  118 }, 
    {  -17,   86 }, {  -25,  101 }, {  -14,   91 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }
  },
  {
    {  -11,   80 }, {  -32,   83 }, {  -19,   89 }, {  -16,   85 }, 
    {  -19,  102 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {  -22,   52 }, 
    {  -48,  123 }, {   -7,   68 }, {  -37,  121 }, {  -58,  164 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {  -19,   45 }, {  -48,  123 }, 
    {  -21,   94 }, {   -9,   73 }, {  -42,  138 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }
  }
};

#if HHI_RQT_ROOT
static const Short
INIT_QT_ROOT_CBF[3][NUM_QT_ROOT_CBF_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 },
  },
  {
    {  -22,   85 }, {  -15,   86 }, {  -13,   84 }, {  -23,  116 },
  },
  {
    {  -36,  103 }, {  -21,   95 }, {  -21,   97 }, {  -24,  114 },
  }
};
#endif
#endif

#if HHI_TRANSFORM_CODING
static const Short
INIT_SIG_FLAG[3][224][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {  -24,  145 }, {  -40,  159 }, {  -38,  155 }, {  -44,  159 }, 
    {   -1,   27 }, {   -4,   68 }, {  -12,  100 }, {  -15,   61 }, 
    {  -14,   82 }, {  -23,  117 }, {    0,    6 }, {  -11,   54 }, 
    {    8,   40 }, {   -1,   65 }, {  -18,  104 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {  -31,  151 }, {  -19,  114 }, {  -22,  124 }, {  -14,  100 }, 
    {   -7,   39 }, {   -1,   61 }, {  -12,   96 }, {  -13,   55 }, 
    {   -1,   61 }, {  -16,  101 }, {    2,    5 }, {    8,   23 }, 
    {    6,   43 }, {    1,   62 }, {  -16,   99 }, {    0,   64 }, 
    {  -19,  119 }, {  -12,   91 }, {  -19,  106 }, {  -25,  108 }, 
    {   19,   -5 }, {  -24,   96 }, {   -7,   78 }, {   19,    1 }, 
    {    1,   54 }, {  -10,   82 }, {    1,   15 }, {   -3,   51 }, 
    {   -4,   62 }, {    1,   61 }, {   -5,   74 }, {    0,   64 }, 
    {  -10,  107 }, {   -9,   90 }, {   -7,   89 }, {  -12,   87 }, 
    {   -3,   32 }, {    0,   60 }, {   -3,   75 }, {   -6,   49 }, 
    {    3,   54 }, {   -5,   78 }, {  -16,   39 }, {   -3,   43 }, 
    {   -6,   64 }, {   -3,   67 }, {   -6,   80 }, {    0,   64 }, 
    {  -14,  112 }, {   -6,   78 }, {   -6,   83 }, {    2,   61 }, 
    {    9,   22 }, {   -6,   65 }, {  -26,  107 }, {    4,   33 }, 
    {    4,   52 }, {  -27,  110 }, {    9,    4 }, {    5,   38 }, 
    {    6,   46 }, {    6,   54 }, {   -3,   72 }, {    0,   64 }, 
    {  -10,   91 }, {   -9,   76 }, {   -3,   61 }, {    1,   46 }, 
    {  -13,   84 }, {  -13,   81 }, {   -2,   60 }, {   -5,   61 }, 
    {  -11,   77 }, {  -10,   76 }, {   -3,   65 }, {   -3,   63 }, 
    {   -4,   59 }, {  -14,   79 }, {    1,   61 }, {  -33,  126 }, 
    {   19,   45 }, {    8,   48 }, {    2,   40 }, {   -3,   40 }, 
    {    8,   46 }, {   16,   31 }, {  -10,   73 }, {  -51,  114 }, 
    {    2,   52 }, {   -9,   71 }, {  -37,  118 }, {   -8,   38 }, 
    {  -10,   64 }, {  -47,  113 }, { -105,  234 }, {  -32,  123 }, 
    {  -11,  105 }, {   -8,   84 }, {   -5,   72 }, {   -7,   71 }, 
    {  -13,   96 }, {   -9,   85 }, {   -4,   75 }, {   -7,   75 }, 
    {  -10,   84 }, {   -7,   81 }, {    0,   76 }, {   -3,   78 }, 
    {  -11,   84 }, {   -4,   75 }, {   -1,   83 }, {    0,   64 }, 
    {   32,   35 }, {    3,   59 }, {   -1,   61 }, {  -31,  104 }, 
    {   29,   28 }, {   11,   51 }, {   -6,   77 }, {  -41,  125 }, 
    {    9,   53 }, {   -3,   76 }, {  -34,  130 }, {  -42,  133 }, 
    {  -29,  109 }, {  -49,  143 }, {  -47,  144 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {  -22,  128 }, {  -14,   96 }, {  -17,   99 }, {  -26,  113 }, 
    {    5,   31 }, {   -2,   65 }, {  -32,  125 }, {  -19,   72 }, 
    {   -7,   69 }, {  -24,  111 }, {    0,   25 }, {   -1,   49 }, 
    {    2,   54 }, {    0,   64 }, {  -15,   97 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   -4,   88 }, {    0,   68 }, {   -4,   75 }, {   -5,   74 }, 
    {   17,    7 }, {    2,   56 }, {   -8,   80 }, {    1,   36 }, 
    {    2,   52 }, {  -11,   85 }, {    6,   12 }, {    5,   41 }, 
    {    2,   54 }, {    0,   65 }, {   -8,   82 }, {    0,   64 }, 
    {   -8,   95 }, {   -7,   77 }, {   -7,   73 }, {   -9,   74 }, 
    {  -18,   77 }, {   -7,   72 }, {  -11,   81 }, {  -29,   92 }, 
    {    4,   47 }, {  -14,   83 }, {  -26,   79 }, {    2,   46 }, 
    {    5,   50 }, {    6,   52 }, {  -22,  104 }, {    0,   64 }, 
    {   -7,   93 }, {   -5,   79 }, {   -6,   79 }, {   -8,   79 }, 
    {    5,   36 }, {    3,   56 }, {  -18,   99 }, {  -10,   59 }, 
    {    0,   56 }, {  -11,   84 }, {   -3,   31 }, {    8,   35 }, 
    {    4,   51 }, {   -1,   66 }, {   -7,   81 }, {    0,   64 }, 
    {    1,   80 }, {  -28,  116 }, {  -26,  112 }, {    2,   56 }, 
    {  -17,   75 }, {    3,   52 }, {  -10,   79 }, {  -26,   91 }, 
    {   -2,   61 }, {   -3,   66 }, {  -45,  114 }, {  -21,   91 }, 
    {    1,   59 }, {    2,   62 }, {  -10,   82 }, {    0,   64 }, 
    {    3,   68 }, {    0,   62 }, {   -8,   69 }, {    1,   43 }, 
    {   -6,   70 }, {   -4,   66 }, {   -2,   60 }, {   -6,   61 }, 
    {  -13,   78 }, {   -8,   71 }, {   -6,   71 }, {   -5,   67 }, 
    {   -7,   63 }, {    0,   56 }, {  -13,   83 }, {   -4,   77 }, 
    {    6,   70 }, {  -19,   94 }, {  -21,   80 }, {   -7,   30 }, 
    {  -15,   88 }, {  -19,   95 }, {  -39,  117 }, {  -10,   42 }, 
    {  -15,   78 }, {  -30,  104 }, {    4,   36 }, {   91, -137 }, 
    {  -45,  125 }, {    0,   42 }, {   30,    2 }, {    0,   64 }, 
    {   -3,   84 }, {   -8,   83 }, {  -13,   86 }, {  -12,   75 }, 
    {   -6,   81 }, {   -9,   83 }, {   -5,   77 }, {   -8,   75 }, 
    {   -8,   79 }, {   -7,   82 }, {    0,   73 }, {   -7,   83 }, 
    {  -13,   84 }, {   -4,   73 }, {   -3,   82 }, {    0,   64 }, 
    {   15,   63 }, {    0,   69 }, {  -28,  110 }, {  -29,  104 }, 
    {   -1,   74 }, {  -22,  105 }, {  -50,  150 }, {  -57,  151 }, 
    {  -21,   96 }, {  -29,  113 }, {  -49,  151 }, {  -85,  197 }, 
    {  -46,  133 }, {  -60,  159 }, {  -84,  198 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {  -34,  146 }, {  -20,  108 }, {  -23,  111 }, {  -38,  141 }, 
    {    3,   34 }, {   -4,   69 }, {  -17,  103 }, {   -9,   50 }, 
    {   -4,   66 }, {  -16,   97 }, {   -1,   24 }, {    0,   50 }, 
    {   -1,   61 }, {  -12,   90 }, {  -14,   97 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {  -12,   99 }, {   -4,   75 }, {   -8,   81 }, {   -7,   80 }, 
    {   15,   13 }, {    2,   57 }, {  -10,   85 }, {    4,   27 }, 
    {  -10,   75 }, {   -9,   81 }, {   -9,   46 }, {    6,   41 }, 
    {    3,   56 }, {   -1,   69 }, {  -14,   96 }, {    0,   64 }, 
    {   -7,   85 }, {  -22,  102 }, {  -12,   79 }, {  -35,  121 }, 
    {   -5,   55 }, {  -16,   88 }, {   -9,   78 }, {  -26,   73 }, 
    {  -28,   99 }, {  -38,  123 }, {   -3,   35 }, {  -25,   97 }, 
    {   -9,   75 }, {    6,   54 }, {  -15,   92 }, {    0,   64 }, 
    {   -9,   94 }, {  -14,   94 }, {   -4,   74 }, {  -14,   92 }, 
    {    7,   37 }, {    3,   57 }, {   -7,   78 }, {   -8,   54 }, 
    {   -7,   70 }, {   -8,   78 }, {   -2,   31 }, {    4,   49 }, 
    {    1,   60 }, {   -5,   75 }, {  -10,   87 }, {    0,   64 }, 
    {   -3,   82 }, {  -19,  102 }, {  -14,   91 }, {  -33,  123 }, 
    {  -46,  124 }, {  -12,   82 }, {  -43,  138 }, {  -46,  121 }, 
    {  -38,  125 }, {  -44,  138 }, {  -32,   89 }, {   -3,   60 }, 
    {  -22,  102 }, {  -27,  114 }, {  -37,  128 }, {    0,   64 }, 
    {    2,   69 }, {  -10,   82 }, {   -1,   57 }, {   -7,   59 }, 
    {   -5,   71 }, {  -13,   85 }, {  -28,  107 }, {  -29,  100 }, 
    {  -21,   93 }, {  -28,  108 }, {   -6,   71 }, {  -48,  136 }, 
    {  -35,  111 }, {  -10,   74 }, {    4,   56 }, {    5,   56 }, 
    {   -3,   84 }, {  -35,  122 }, {  -42,  111 }, {  101, -147 }, 
    {  -14,   87 }, {  -70,  179 }, {    6,   30 }, {    0,   64 }, 
    {    8,   32 }, {  -64,  156 }, {    0,   64 }, {    0,   64 }, 
    {  -65,  144 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    4,   67 }, {   -5,   77 }, {  -12,   85 }, {  -28,  105 }, 
    {   -5,   77 }, {   -7,   79 }, {   -7,   79 }, {  -28,  109 }, 
    {  -17,   94 }, {  -11,   88 }, {   -6,   81 }, {  -32,  125 }, 
    {  -38,  128 }, {  -40,  136 }, {  -39,  142 }, {    0,   64 }, 
    {  -35,  148 }, {  -47,  150 }, {  -22,  100 }, {  -73,  176 }, 
    {  -12,   91 }, {  -54,  155 }, {  -86,  203 }, {  -71,  174 }, 
    {  -66,  167 }, {  -41,  129 }, { -144,  285 }, { -168,  304 }, 
    {  -59,  148 }, { -125,  248 }, { -129,  252 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  }
};

static const Short
INIT_LAST_FLAG[3][224][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   24,  -37 }, {   35,  -52 }, {    8,   -9 }, {   14,  -11 }, 
    {   14,  -10 }, {   22,  -18 }, {   48,  -64 }, {   -1,   29 }, 
    {   20,   13 }, {  -15,   51 }, {   77, -101 }, {   22,  -20 }, 
    {  -34,   89 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   33,  -47 }, {   38,  -54 }, {   25,  -32 }, {   46,  -66 }, 
    {   24,  -27 }, {   26,  -28 }, {   36,  -39 }, {   -6,   48 }, 
    {   -2,   51 }, {   -1,   44 }, {    9,   26 }, {  -36,   93 }, 
    {  -20,   61 }, {  -51,  130 }, {   29,   -2 }, {    0,    0 }, 
    {   42,  -30 }, {   43,  -46 }, {   61,  -81 }, {   48,  -51 }, 
    {   21,    5 }, {    8,   22 }, {    5,   38 }, {   23,   12 }, 
    {  102, -138 }, {  304, -440 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   30,  -24 }, {   24,  -15 }, {   19,   -2 }, {    9,   15 }, 
    {    9,   16 }, {    5,   27 }, {   19,    6 }, {   11,   29 }, 
    {   -7,   62 }, {   -9,   66 }, {    7,   41 }, {  -11,   76 }, 
    {  -42,  130 }, {  -14,   91 }, {   16,   14 }, {    0,   64 }, 
    {   22,   14 }, {   26,   -1 }, {   13,   18 }, {    4,   37 }, 
    {    5,   36 }, {   20,   14 }, {   12,   29 }, {   24,   17 }, 
    {  -50,  133 }, { -165,  315 }, { -160,  301 }, { -217,  408 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   31,  -12 }, {   20,    4 }, {   16,    9 }, {   16,    9 }, 
    {   17,    8 }, {   20,    4 }, {   23,    4 }, {   17,   18 }, 
    {    9,   36 }, {   10,   36 }, {   10,   39 }, {    7,   52 }, 
    {   -3,   75 }, {  -16,  103 }, {    0,   64 }, {    0,   64 }, 
    {   14,   44 }, {   -2,   64 }, {    3,   51 }, {   18,   22 }, 
    {   -8,   70 }, {    3,   53 }, {    7,   48 }, {   -6,   70 }, 
    {  -48,  133 }, {  -22,   96 }, { -111,  240 }, {  -55,  156 }, 
    {  -92,  215 }, { -347,  585 }, {    0,   64 }, {    0,   64 }, 
    {   12,   26 }, {   11,   23 }, {   16,   15 }, {   19,   17 }, 
    {    9,   30 }, {   14,   24 }, {   18,   21 }, {   16,   30 }, 
    {   13,   27 }, {   12,   33 }, {   13,   37 }, {    9,   54 }, 
    {   12,   37 }, {   12,   42 }, {   15,   49 }, {    0,   64 }, 
    {   23,   26 }, {   17,   30 }, {    0,   60 }, {    7,   53 }, 
    {   18,   33 }, {   15,   39 }, {   10,   48 }, {  -22,  101 }, 
    {   19,   34 }, {   10,   51 }, {  -35,  124 }, {  -16,   93 }, 
    {    2,   65 }, {   -9,   83 }, {  -24,  109 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   17,   18 }, {   28,  -21 }, {   40,  -50 }, {    7,   20 }, 
    {   -9,   47 }, {  -12,   43 }, {    9,   -1 }, {   -7,   52 }, 
    {  -29,   89 }, {   25,  -20 }, {  -36,   82 }, {  106, -178 }, 
    {  248, -404 }, {   56,  -38 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   20,   10 }, {   25,   -8 }, {   23,   -5 }, {    2,   32 }, 
    {    8,   15 }, {   19,   -5 }, {   15,    0 }, {    8,   27 }, 
    {   -1,   48 }, {    1,   42 }, {    1,   41 }, {  -48,  124 }, 
    {  -97,  213 }, {   32,  -21 }, {   61,  -47 }, { -480,  845 }, 
    {   22,   17 }, {   26,   -6 }, {   42,  -38 }, {   33,  -15 }, 
    {    9,   29 }, {  -19,   70 }, {  -77,  167 }, {  -80,  176 }, 
    {  -96,  186 }, {  171, -294 }, {    5,   45 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   23,    2 }, {   30,  -12 }, {   20,    9 }, {   16,   16 }, 
    {   19,    4 }, {    8,   24 }, {   15,   13 }, {   10,   26 }, 
    {   12,   25 }, {  -12,   66 }, {   -7,   60 }, {   -8,   64 }, 
    {   -2,   63 }, {   -7,   75 }, { -116,  252 }, {    0,   64 }, 
    {   -4,   68 }, {   20,   16 }, {   17,   22 }, {   13,   31 }, 
    {    3,   48 }, {  -21,   84 }, {  -41,  117 }, {  -43,  125 }, 
    {    5,   40 }, {   29,    4 }, {  -24,   87 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   14,   23 }, {   24,    4 }, {   15,   23 }, {    7,   34 }, 
    {   19,   14 }, {   19,   12 }, {   17,   18 }, {   11,   33 }, 
    {    4,   44 }, {   -2,   55 }, {    2,   54 }, {    3,   53 }, 
    {  -43,  137 }, {  -22,  104 }, {    0,   64 }, {    0,   64 }, 
    {   -5,   76 }, {    7,   50 }, {    9,   43 }, {    6,   50 }, 
    {    0,   59 }, {    1,   56 }, {    4,   51 }, {   22,   18 }, 
    {   21,   21 }, { -105,  242 }, {  -52,  145 }, {  260, -439 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   50 }, {    5,   42 }, {    4,   43 }, {   18,   20 }, 
    {   17,   23 }, {    0,   55 }, {   10,   39 }, {    6,   46 }, 
    {   10,   36 }, {    9,   41 }, {   12,   42 }, {    3,   63 }, 
    {   15,   30 }, {    9,   46 }, {    0,   73 }, {    0,   64 }, 
    {   12,   47 }, {   11,   43 }, {    5,   50 }, {    5,   53 }, 
    {   -2,   68 }, {  -11,   83 }, {  -18,   93 }, {  -59,  154 }, 
    {  -22,   99 }, {  -45,  137 }, {  -63,  167 }, {  -56,  152 }, 
    {  -34,  117 }, {  -45,  136 }, {  -74,  182 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   20,    7 }, {   38,  -44 }, {   37,  -43 }, {   -9,   48 }, 
    {  -45,  104 }, {  -87,  174 }, {   26,  -35 }, {   51,  -63 }, 
    {  129, -174 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    7,   38 }, {   28,  -14 }, {   25,   -7 }, {    2,   38 }, 
    {  -13,   53 }, {   11,    2 }, {  -18,   59 }, {   23,   -4 }, 
    {   46,  -35 }, {   -9,   59 }, {  -32,   97 }, {   -5,   57 }, 
    {  -98,  215 }, {  -51,  141 }, {  163, -267 }, {    0,    0 }, 
    {    2,   48 }, {   57,  -65 }, {   70,  -78 }, {   74,  -74 }, 
    {   38,   -7 }, {   64,  -44 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   12,   24 }, {   17,   17 }, {   11,   31 }, {    6,   35 }, 
    {    9,   25 }, {   -2,   41 }, {   11,   19 }, {   -3,   51 }, 
    {  -30,  100 }, {   -4,   55 }, {    2,   49 }, {   38,  -17 }, 
    {   43,  -28 }, {  -35,  117 }, {  -97,  214 }, {    0,   64 }, 
    {    7,   41 }, {   29,   -3 }, {   17,   23 }, {    6,   45 }, 
    {   12,   36 }, {  -22,   85 }, {    2,   54 }, {  -18,   87 }, 
    {    0,   65 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   -5,   59 }, {   14,   25 }, {   13,   29 }, {    4,   42 }, 
    {    8,   33 }, {    5,   37 }, {   20,   11 }, {   16,   20 }, 
    {   15,   22 }, {  -12,   70 }, {   -9,   67 }, {   13,   35 }, 
    {  -15,   85 }, {  -25,  101 }, {    0,   64 }, {    0,   64 }, 
    {   -3,   65 }, {    6,   50 }, {    3,   56 }, {   21,   19 }, 
    {   -1,   64 }, {  -73,  175 }, {  -17,   76 }, {    7,   52 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   12,   22 }, {   15,   20 }, {   18,   14 }, {   13,   28 }, 
    {   18,   18 }, {    0,   54 }, {  -14,   77 }, {  -34,  113 }, 
    {    4,   43 }, {    7,   43 }, {    2,   55 }, {   -2,   69 }, 
    {  -17,   82 }, {   -2,   60 }, {   -8,   81 }, {    0,   64 }, 
    {  -20,   92 }, {    1,   56 }, {  -35,  114 }, {  -12,   78 }, 
    {    6,   51 }, {  -48,  141 }, {  -77,  183 }, {  -63,  157 }, 
    {    7,   46 }, {  -56,  149 }, { -125,  254 }, { -134,  247 }, 
    {  -49,  134 }, {  -72,  167 }, { -130,  253 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  }
};

static const Short
INIT_ABS_GREATER_ONE_FLAG[3][80][2] =
{
  {
    {  -11,   87 }, {  -20,   64 }, {  -16,   68 }, {  -13,   71 }, 
    {  -10,   73 }, {   -5,   67 }, {   -8,   26 }, {   -8,   37 }, 
    {   -3,   36 }, {   -9,   56 }, {    0,   63 }, {   -5,   39 }, 
    {  -12,   56 }, {   -9,   57 }, {   -1,   52 }, {   -4,   72 }, 
    {  -19,   73 }, {  -28,   88 }, {  -23,   85 }, {   -3,   59 }, 
    {   -2,   72 }, {  -27,   97 }, {  -22,   89 }, {  -14,   77 }, 
    {   -1,   58 }, {  -10,   86 }, {  -22,   74 }, {  -13,   63 }, 
    {   -6,   57 }, {   -5,   63 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   -4,   70 }, {   16,   -2 }, {    9,   23 }, {    5,   41 }, 
    {   -7,   67 }, {   -6,   63 }, {   13,   -5 }, {  -12,   42 }, 
    {  -18,   53 }, {  -14,   59 }, {   -5,   65 }, {   -8,   36 }, 
    {  -22,   67 }, {  -35,   89 }, {   -3,   47 }, {  -12,   77 }, 
    {  -20,   66 }, {  -51,  113 }, {  -44,  109 }, {  -23,   84 }, 
    {   -1,   64 }, {  -58,  127 }, {  -71,  143 }, {  -67,  134 }, 
    {  -91,  187 }, {   -1,   64 }, {   20,   -3 }, {   13,   21 }, 
    {    2,   46 }, {    5,   48 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {   -4,   71 }, {   29,   -5 }, {    1,   45 }, {   -6,   58 }, 
    {   -9,   67 }, {   -6,   66 }, {   23,   -7 }, {    3,   26 }, 
    {   -5,   42 }, {    5,   31 }, {  -12,   79 }, {   -2,   45 }, 
    {  -15,   67 }, {  -11,   62 }, {   -3,   54 }, {   -4,   70 }, 
    {  -13,   68 }, {  -31,  100 }, {  -26,   91 }, {  -18,   84 }, 
    {   -9,   83 }, {  -30,  103 }, {  -24,   90 }, {  -43,  122 }, 
    {   -6,   66 }, {    4,   59 }, {   34,  -11 }, {   21,   10 }, 
    {   14,   25 }, {    5,   44 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   -4,   69 }, {   40,  -33 }, {   19,    8 }, {  -11,   65 }, 
    {   -6,   59 }, {  -11,   68 }, {   12,   -2 }, {   17,  -10 }, 
    {   -6,   34 }, {   11,   14 }, {  -11,   70 }, {    1,   27 }, 
    {  -26,   71 }, {  -28,   79 }, {   -2,   47 }, {    6,   47 }, 
    {  -23,   67 }, {  -47,  107 }, {  -55,  117 }, {  -21,   83 }, 
    {    1,   59 }, {  -38,   82 }, {  -34,   77 }, {  -45,   95 }, 
    {   10,   25 }, {    9,   46 }, {   41,  -31 }, {   32,  -17 }, 
    {   14,   19 }, {   17,   18 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {   -2,   65 }, {   18,   22 }, {   10,   33 }, {   -2,   55 }, 
    {   -7,   64 }, {   -6,   67 }, {   19,   11 }, {   -5,   50 }, 
    {   -7,   53 }, {   -4,   54 }, {  -23,   99 }, {   -3,   51 }, 
    {    2,   41 }, {  -32,  102 }, {  -16,   79 }, {   -8,   77 }, 
    {  -21,   84 }, {  -26,   91 }, {  -33,  104 }, {   -4,   61 }, 
    {  -31,  122 }, {  -34,  110 }, {  -25,   96 }, {  -43,  124 }, 
    {   -6,   70 }, {    3,   60 }, {   23,   12 }, {   12,   30 }, 
    {   11,   33 }, {    8,   40 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   -2,   66 }, {   40,  -20 }, {    0,   46 }, {   -5,   54 }, 
    {    7,   37 }, {  -39,  116 }, {   31,  -27 }, {    1,   22 }, 
    {  -35,   82 }, {  -32,   85 }, {  -15,   72 }, {   16,    0 }, 
    {  -43,  102 }, {  -75,  152 }, {   -8,   55 }, {   -9,   68 }, 
    {  -12,   54 }, {  -84,  171 }, {  -93,  186 }, {   25,   12 }, 
    { -104,  222 }, {  -40,   92 }, {  -51,   93 }, {  110, -169 }, 
    {    3,   52 }, {   17,   33 }, {   55,  -45 }, {   28,    1 }, 
    {   -5,   55 }, {   29,   -4 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  }
};

static const Short
INIT_COEFF_LEVEL_MINUS_ONE_FLAG[3][80][2] =
{
  {
    {  -12,   72 }, {  -10,   79 }, {  -11,   87 }, {  -14,   94 }, 
    {  -35,  136 }, {  -10,   58 }, {   -1,   54 }, {  -17,   86 }, 
    {   -5,   70 }, {  -22,  105 }, {  -13,   70 }, {   -2,   59 }, 
    {  -13,   81 }, {  -21,   96 }, {   -3,   73 }, {  -24,   90 }, 
    {  -19,   88 }, {   -1,   63 }, {    1,   60 }, {  -17,   97 }, 
    {   -7,   69 }, {  -20,   95 }, {  -11,   84 }, {  -27,  110 }, 
    {  -12,   96 }, {   -9,   72 }, {   -8,   76 }, {   -4,   75 }, 
    {   -3,   76 }, {   -9,   94 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {  -11,   65 }, {   -4,   66 }, {   -9,   82 }, {  -24,  107 }, 
    {  -24,  111 }, {  -18,   58 }, {  -10,   61 }, {  -19,   82 }, 
    {  -28,   97 }, {    6,   47 }, {  -12,   50 }, {  -17,   73 }, 
    {   -8,   66 }, {  -44,  115 }, {  -17,   86 }, {  -26,   74 }, 
    {   -4,   48 }, {  -43,  115 }, {  -58,  141 }, {  -51,  137 }, 
    {  -51,  117 }, {  -85,  176 }, {  -47,  120 }, {  -95,  202 }, 
    {  -65,  159 }, {   -2,   53 }, {    7,   43 }, {   -2,   67 }, 
    {    3,   62 }, {    4,   67 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {   -2,   49 }, {  -13,   81 }, {  -11,   82 }, {  -13,   88 }, 
    {  -24,  112 }, {   -4,   44 }, {  -17,   77 }, {  -16,   82 }, 
    {  -19,   89 }, {  -28,  110 }, {  -12,   64 }, {  -11,   70 }, 
    {  -18,   88 }, {   -6,   67 }, {  -19,   97 }, {  -17,   76 }, 
    {  -27,  100 }, {  -18,   88 }, {  -13,   84 }, {  -17,   94 }, 
    {  -11,   73 }, {  -15,   83 }, {  -10,   77 }, {  -10,   80 }, 
    {  -12,   91 }, {   -8,   63 }, {   -7,   71 }, {   -6,   73 }, 
    {   -8,   80 }, {   -9,   90 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   14,   20 }, {   -7,   68 }, {   -4,   66 }, {   -8,   76 }, 
    {  -50,  148 }, {    2,   22 }, {   11,   15 }, {   -4,   49 }, 
    {  -41,  117 }, {  -60,  149 }, {  -12,   49 }, {   -6,   55 }, 
    {    1,   44 }, {   -5,   58 }, {  -12,   73 }, {   13,    9 }, 
    {    5,   26 }, {  -29,  101 }, {    0,   46 }, {  -23,   92 }, 
    {  -12,   57 }, {    6,   30 }, {   35,   -4 }, {   42,  -24 }, 
    {   22,   24 }, {   24,    0 }, {   11,   34 }, {    2,   61 }, 
    {   -5,   75 }, {    4,   62 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  },
  {
    {    0,   43 }, {   -6,   65 }, {   -7,   71 }, {  -31,  117 }, 
    {  -25,  109 }, {  -20,   76 }, {  -14,   73 }, {  -20,   88 }, 
    {  -21,   92 }, {   -6,   71 }, {  -19,   73 }, {  -34,  108 }, 
    {  -27,  101 }, {   -7,   69 }, {   -7,   75 }, {  -18,   77 }, 
    {   -7,   64 }, {  -20,   91 }, {   -9,   75 }, {   -7,   78 }, 
    {  -26,   98 }, {  -13,   81 }, {   -6,   69 }, {  -12,   83 }, 
    {   -2,   70 }, {   -3,   50 }, {   -6,   66 }, {   -7,   73 }, 
    {   -6,   75 }, {  -11,   91 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   19,   10 }, {    2,   49 }, {    4,   52 }, {   -4,   69 }, 
    {  -32,  114 }, {   13,   -1 }, {  -29,   85 }, {  -64,  143 }, 
    {  -90,  186 }, {   26,    4 }, {  -24,   68 }, {    8,   30 }, 
    {  -10,   61 }, {  -16,   78 }, {  -27,   96 }, {   31,  -16 }, 
    {  -12,   54 }, {  -15,   70 }, {  -68,  158 }, {   31,   12 }, 
    {   -8,   44 }, {  -32,   63 }, {  -36,   81 }, {  -53,  106 }, 
    {   36,   12 }, {   26,   -5 }, {   19,   17 }, {  -22,  101 }, 
    {    5,   54 }, {    8,   53 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 } 
  }
};
#else
// initial probability for sigmap
static const Short
INIT_SIGMAP[3][210][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {  -32,  130 }, {  -32,  114 }, 
    {  -59,  162 }, {   -2,   50 }, {  -12,   61 }, {  -11,   53 }, 
    {   -8,   32 }, {    9,    3 }, {   12,   -6 }, {   -7,   21 }, 
    {    5,   14 }, {  -15,   34 }, {  -19,   57 }, {    9,   14 }, 
    {    0,    0 }, {  -14,   88 }, {  -37,  108 }, {  -27,   97 }, 
    {  -25,   76 }, {  -16,   64 }, {  -22,   61 }, {  192, -264 }, 
    { -224,  362 }, {    0,    0 }, {    0,    0 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {  -14,   98 }, {  -17,   89 }, {  -14,   85 }, {  -22,   88 }, 
    {    1,   46 }, {  -11,   64 }, {  -39,   94 }, {   -6,   43 }, 
    {  -19,   61 }, {    2,   29 }, {   18,    2 }, {  -43,   98 }, 
    {  -35,  101 }, {   -8,   55 }, {   11,   -1 }, {   -1,   70 }, 
    {  -15,   80 }, {  -32,  111 }, {   -4,   51 }, {    6,   40 }, 
    {  -23,   76 }, {  -70,  136 }, {  -58,  134 }, {   12,   -4 }, 
    {   67,  -95 }, {    0,    0 }, {  -13,   28 }, {   58,  -74 }, 
    {    0,    0 }, {  296, -425 }, {  -11,   96 }, {  -14,   88 }, 
    {  -11,   81 }, {   -7,   64 }, {  -19,   81 }, {  -12,   70 }, 
    {  -26,   88 }, {  -11,   63 }, {    0,   40 }, {    7,   26 }, 
    {  -10,   51 }, {  -26,   86 }, {  -29,   98 }, {  -10,   64 }, 
    {  -10,   36 }, {   27,   31 }, {    3,   56 }, {    2,   55 }, 
    {  -13,   72 }, {   11,   33 }, {    3,   44 }, {  -19,   67 }, 
    {   -3,   56 }, {   13,    8 }, {   51,  -64 }, {    4,   -2 }, 
    {  -39,   93 }, {  -21,   50 }, {  -75,  137 }, {   92, -129 }, 
    {   -9,  102 }, {  -12,   90 }, {  -12,   94 }, {   -7,   74 }, 
    {  -14,   84 }, {   -6,   70 }, {  -17,   84 }, {  -10,   72 }, 
    {   -2,   58 }, {    2,   50 }, {   -4,   50 }, {  -14,   81 }, 
    {  -16,   82 }, {  -20,   83 }, {   -3,   52 }, {   30,   37 }, 
    {    2,   62 }, {    4,   65 }, {   -1,   60 }, {    7,   44 }, 
    {   19,   24 }, {  -16,   69 }, {    7,   45 }, {  -10,   64 }, 
    {   -9,   54 }, {  -11,   41 }, {  -16,   71 }, {  -43,  122 }, 
    {  -53,  120 }, {   -9,   30 }, {  -13,  109 }, {  -10,   88 }, 
    {  -15,   99 }, {  -13,   85 }, {  -11,   89 }, {   -5,   78 }, 
    {   -7,   71 }, {   -6,   76 }, {   -8,   82 }, {  -11,   85 }, 
    {   -8,   75 }, {   -4,   79 }, {   -8,   84 }, {   -6,   77 }, 
    {   -1,   90 }, {   29,   40 }, {    1,   63 }, {   25,   35 }, 
    {    3,   55 }, {    7,   56 }, {    9,   53 }, {  -33,  105 }, 
    {  -19,   94 }, {   -7,   77 }, {   -5,   74 }, {  -15,   82 }, 
    {  -12,   92 }, {  -47,  142 }, {  -32,  113 }, {  -55,  162 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   -5,   82 }, {  -26,  106 }, 
    {  -15,   83 }, {   -7,   52 }, {  -34,  110 }, {  -20,   66 }, 
    {  -27,   74 }, {  -10,   37 }, {   -7,   16 }, {   -6,   13 }, 
    {  -13,   40 }, {  -18,   49 }, {   24,  -16 }, {   59,  -76 }, 
    {   -5,   10 }, {    5,   62 }, {  -29,  100 }, {  -39,  120 }, 
    {  -48,  115 }, {  -50,  125 }, {  -44,   92 }, {  -89,  150 }, 
    { -130,  220 }, {    0,    0 }, {    0,    0 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    5,   61 }, {   -6,   70 }, {   -9,   68 }, {  -18,   76 }, 
    {  -15,   80 }, {    1,   45 }, {  -24,   80 }, {  -10,   56 }, 
    {  -16,   53 }, {  -13,   42 }, {  -17,   46 }, {  -28,   85 }, 
    {   -3,   51 }, {  -12,   62 }, {   -8,   19 }, {    2,   67 }, 
    {  -14,   83 }, {  -30,  107 }, {  -39,  108 }, {  -27,  103 }, 
    {   -1,   42 }, {  -54,  128 }, {  -62,  130 }, {  -12,   26 }, 
    {    0,    0 }, {    0,    0 }, {  -10,   46 }, {   67,  -83 }, 
    {   28,   12 }, {    0,    0 }, {    5,   66 }, {   -3,   72 }, 
    {  -11,   81 }, {  -13,   76 }, {   -6,   64 }, {   -7,   66 }, 
    {  -12,   68 }, {  -11,   67 }, {    0,   45 }, {    2,   36 }, 
    {  -28,   81 }, {   -2,   48 }, {   -3,   50 }, {    1,   44 }, 
    {  -30,   83 }, {    4,   68 }, {    3,   59 }, {   -4,   65 }, 
    {   -3,   54 }, {  -13,   82 }, {  -14,   76 }, {  -55,  132 }, 
    {  -60,  152 }, {  -23,   50 }, {    6,   -7 }, {  -12,   22 }, 
    {  -28,   87 }, {    8,   27 }, {   32,  -30 }, {    0,    0 }, 
    {    3,   76 }, {  -13,   94 }, {  -21,  105 }, {  -23,   95 }, 
    {  -11,   81 }, {    1,   62 }, {  -25,   97 }, {  -17,   84 }, 
    {   -6,   64 }, {   -4,   60 }, {  -14,   68 }, {   -9,   70 }, 
    {  -11,   70 }, {   -6,   56 }, {   -9,   63 }, {    5,   73 }, 
    {    3,   61 }, {   -2,   71 }, {   -8,   71 }, {   -3,   65 }, 
    {    9,   46 }, {  -41,  115 }, {   11,   39 }, {  -16,   76 }, 
    {  -39,  102 }, {  -27,   65 }, {  -40,  111 }, {    0,   47 }, 
    {   16,    6 }, {   36,  -44 }, {    1,   78 }, {   -4,   79 }, 
    {   -6,   82 }, {  -28,  110 }, {  -11,   88 }, {   -8,   81 }, 
    {  -16,   81 }, {   -7,   78 }, {  -11,   88 }, {  -17,   94 }, 
    {  -10,   77 }, {   -3,   79 }, {   -6,   78 }, {   -8,   80 }, 
    {  -10,   99 }, {   13,   62 }, {    2,   65 }, {  -14,   95 }, 
    {  -22,   94 }, {  -21,  100 }, {   -3,   74 }, {  -10,   69 }, 
    {  -11,   82 }, {  -21,  100 }, {  -47,  134 }, {  -55,  143 }, 
    {  -52,  154 }, {  -53,  151 }, {  -65,  165 }, {  -74,  187 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   -8,   89 }, {  -25,  106 }, 
    {  -29,  108 }, {  -34,   97 }, {    2,   55 }, {  -18,   62 }, 
    {   52,  -60 }, {   44,  -49 }, {  -28,   58 }, {  -20,   40 }, 
    {    7,   14 }, {    9,   10 }, {  -26,   62 }, { -218,  404 }, 
    {    0,   13 }, {  -30,  136 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   73 }, {   -2,   66 }, {   -7,   66 }, {  -34,  107 }, 
    {  -46,  142 }, {   -2,   51 }, {  -67,  160 }, {  -59,  144 }, 
    {   -6,   25 }, {    1,    7 }, {   43,  -54 }, {  -65,  151 }, 
    {   -9,   55 }, {   18,    6 }, {   14,  -18 }, {   10,   51 }, 
    {  -17,   89 }, {  -25,   95 }, {  -48,  122 }, {  -28,  105 }, 
    {   19,   12 }, {   -3,   48 }, { -121,  214 }, {    0,    0 }, 
    {    0,    0 }, {    0,    0 }, {  136, -182 }, { -176,  358 }, 
    {    0,   64 }, {    0,   64 }, {    2,   70 }, {   -1,   70 }, 
    {   -6,   71 }, {   -9,   67 }, {   -5,   70 }, {   -1,   59 }, 
    {  -56,  144 }, {  -13,   74 }, {    5,   32 }, {    7,   16 }, 
    {   20,    0 }, {  -54,  137 }, {  -41,  118 }, {  -27,   96 }, 
    {   -1,   32 }, {   10,   55 }, {   -4,   74 }, {  -10,   78 }, 
    {  -42,  123 }, {  -65,  175 }, {  -54,  146 }, {    0,   43 }, 
    {  -97,  207 }, {    0,    0 }, {   30,  -45 }, {    0,    0 }, 
    {   49,  -57 }, {   72,  -89 }, {    0,   64 }, {    0,   64 }, 
    {    1,   72 }, {  -13,   92 }, {  -14,   94 }, {  -15,   85 }, 
    {   -4,   69 }, {    1,   64 }, {  -39,  122 }, {  -23,  100 }, 
    {   -8,   69 }, {   -1,   54 }, {  -11,   64 }, {  -43,  130 }, 
    {  -50,  137 }, {  -47,  129 }, {  -36,  108 }, {   -1,   77 }, 
    {   -8,   80 }, {  -23,  107 }, {  -34,  114 }, {  -17,   90 }, 
    {   -2,   65 }, {  -61,  140 }, {  -50,  132 }, { -111,  218 }, 
    {  -11,   56 }, {   20,  -16 }, {   -5,   47 }, {  131, -183 }, 
    {   31,  -42 }, {    0,    0 }, {   -1,   76 }, {   -9,   85 }, 
    {  -12,   91 }, {  -21,   97 }, {  -21,  105 }, {  -20,  104 }, 
    {  -28,  104 }, {  -24,  108 }, {  -38,  136 }, {  -41,  134 }, 
    {  -23,  101 }, {  -16,   96 }, {  -24,  109 }, {  -39,  132 }, 
    {  -32,  136 }, {  -21,  114 }, {  -41,  138 }, {  -55,  162 }, 
    {  -55,  146 }, {  -62,  165 }, {  -60,  168 }, {  -66,  162 }, 
    {  -77,  187 }, {  -78,  188 }, {  -30,  104 }, {  -69,  160 }, 
    { -119,  249 }, {  -74,  183 }, { -152,  293 }, { -138,  271 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  }
};

// initial probability for lastbit
static const Short
INIT_LAST_FLAG[3][210][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   28,  -46 }, {   36,  -58 }, 
    {   30,  -25 }, {    9,   36 }, {  -44,   96 }, {   39,  -37 }, 
    { -127,  216 }, {  -50,  115 }, {  -96,  194 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {   55,  -59 }, {   25,  -26 }, {   23,    8 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   31,  -40 }, {   42,  -57 }, {   42,  -42 }, {    0,   42 }, 
    {  -49,  117 }, {  -38,   90 }, {  -80,  168 }, {  -78,  152 }, 
    {  -31,  106 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {   51,  -49 }, 
    {   36,  -33 }, {   28,   -1 }, { -258,  441 }, {   80,  -70 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   28,  -23 }, {   20,   -4 }, 
    {   16,   15 }, {   -6,   58 }, {    1,   51 }, {    6,   39 }, 
    {  -18,   84 }, {   -7,   67 }, {   -2,   68 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {   17,   24 }, {   21,   10 }, {   13,   31 }, 
    {   -3,   54 }, {  -94,  181 }, { -180,  315 }, { -397,  664 }, 
    { -347,  585 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   29,  -15 }, {   15,   10 }, {   21,    5 }, {   11,   29 }, 
    {   11,   32 }, {    8,   43 }, {   11,   42 }, {    8,   55 }, 
    {    2,   74 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {   14,   40 }, 
    {    8,   44 }, {   11,   41 }, {  -21,   95 }, {  -12,   83 }, 
    {  -79,  183 }, { -111,  241 }, {  -37,  132 }, {   48,  -18 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   13,   23 }, {   11,   21 }, 
    {   10,   27 }, {   16,   16 }, {   13,   22 }, {   13,   24 }, 
    {   21,   13 }, {   17,   21 }, {   14,   26 }, {   13,   36 }, 
    {   11,   38 }, {   12,   37 }, {   18,   34 }, {    8,   51 }, 
    {   15,   51 }, {   25,   22 }, {   23,   21 }, {   19,   32 }, 
    {   18,   31 }, {   20,   30 }, {   14,   42 }, {    6,   57 }, 
    {   15,   40 }, {  -14,   89 }, {   13,   52 }, {  -16,   91 }, 
    {   -8,   78 }, {    3,   70 }, {   -6,   78 }, {  -34,  129 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   18,   11 }, {   26,  -23 }, 
    {  -22,   77 }, {  -60,  148 }, {   15,  -10 }, {   40,  -47 }, 
    {  -29,   73 }, {  -48,  136 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {   49,  -73 }, {   42,  -59 }, {  -91,  162 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   19,    1 }, {   21,    2 }, {   20,    3 }, {  -16,   70 }, 
    {   -1,   42 }, {  -26,   87 }, {   -8,   56 }, {  -36,  102 }, 
    {  -55,  143 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {   19,   15 }, 
    {   31,  -22 }, {  -40,  108 }, {   20,  -11 }, {   73, -109 }, 
    {   17,   28 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   23,   -5 }, {   26,   -8 }, 
    {   26,   -5 }, {    2,   41 }, {  -10,   61 }, {  -31,   96 }, 
    {    5,   43 }, {   -3,   61 }, {  -18,   96 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {   10,   37 }, {   13,   29 }, {    1,   54 }, 
    {   30,  -11 }, {   27,   -8 }, { -136,  247 }, {   66,  -73 }, 
    {    0,   65 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   19,    6 }, {   21,    8 }, {   20,   10 }, {   10,   31 }, 
    {    7,   39 }, {    5,   45 }, {    8,   42 }, {    1,   62 }, 
    {   -5,   80 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    4,   55 }, 
    {   13,   37 }, {    8,   44 }, {    7,   47 }, {  -27,   97 }, 
    {   23,   24 }, {   -8,   67 }, {  -11,   82 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    1,   44 }, {   12,   25 }, 
    {   15,   21 }, {   14,   20 }, {   10,   32 }, {   12,   33 }, 
    {   19,   20 }, {    9,   37 }, {    8,   41 }, {    9,   41 }, 
    {    7,   45 }, {    8,   46 }, {   12,   43 }, {    2,   61 }, 
    {    2,   71 }, {   11,   44 }, {    9,   46 }, {   -3,   66 }, 
    {  -21,   95 }, {   -5,   69 }, {    6,   54 }, {    8,   50 }, 
    {  -22,   98 }, {   -5,   72 }, {  -20,   96 }, {  -49,  138 }, 
    {  -49,  142 }, {  -64,  172 }, {  -76,  188 }, {  -50,  154 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   32,  -26 }, {   55,  -79 }, 
    {  -60,  126 }, {  277, -443 }, { -400,  707 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {   85, -134 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {   16,    6 }, {   19,    6 }, {   35,  -32 }, {   45,  -41 }, 
    {    4,   32 }, {  -15,   67 }, { -186,  375 }, { -150,  308 }, 
    {   14,    7 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {   28,   -9 }, 
    {   43,  -38 }, {  -16,   67 }, { -160,  325 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   17,    6 }, {   17,   14 }, 
    {   14,   18 }, {   12,   26 }, {   -3,   52 }, {   -7,   58 }, 
    {   26,    1 }, {   72,  -79 }, {   17,   29 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {   22,    6 }, {   21,   14 }, {    4,   48 }, 
    { -128,  274 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    1,   43 }, {   20,   12 }, {   21,    8 }, {   19,   12 }, 
    {   15,   22 }, {   15,   26 }, {  -20,   82 }, {    1,   57 }, 
    {    0,   65 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {  -12,   76 }, 
    {   11,   41 }, {   -6,   68 }, {   14,   36 }, { -120,  217 }, 
    {    0,    0 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    4,   36 }, {    6,   36 }, 
    {    3,   42 }, {   -1,   48 }, {    3,   43 }, {   17,   19 }, 
    {  -12,   69 }, {  -17,   80 }, {   -4,   60 }, {   -2,   56 }, 
    {  -24,   96 }, {   -7,   68 }, {    9,   44 }, {  -22,   99 }, 
    {  -22,  106 }, {  -11,   76 }, {  -21,   95 }, {    3,   55 }, 
    {  -34,  112 }, {  -52,  141 }, {  -29,  107 }, {    1,   64 }, 
    {  -51,  143 }, {  -74,  174 }, {  -79,  185 }, {  -87,  194 }, 
    {  -79,  183 }, { -141,  283 }, {  -99,  211 }, {  -75,  184 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  }
};

// initial probability for greater than one
static const Short
INIT_ONE_FLAG[3][70][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   -7,   74 }, {   -4,   26 }, 
    {   -9,   33 }, {   -5,   28 }, {   32,  -35 }, {    3,   47 }, 
    {   35,  -33 }, {   56,  -73 }, {    8,   -2 }, {   29,  -26 }, 
    {   -1,   63 }, {    1,   19 }, {   23,  -17 }, {   22,  -18 }, 
    {   14,    7 }, {   -1,   59 }, {   37,  -37 }, {   12,    5 }, 
    {    2,   21 }, {    9,   17 }, {   -6,   72 }, {   -7,   31 }, 
    {   -4,   35 }, {    3,   21 }, {   -1,   40 }, {    2,   57 }, 
    {   14,   -1 }, {   25,   -5 }, {   19,    5 }, {   12,   22 }, 
    {   -8,   77 }, {   -4,   23 }, {   -4,   34 }, {   -7,   47 }, 
    {   -6,   55 }, {   10,   39 }, {   19,   -7 }, {   12,   14 }, 
    {   10,   29 }, {    9,   35 }, {  -12,   87 }, {  -22,   66 }, 
    {  -18,   71 }, {  -15,   73 }, {  -13,   77 }, {   -2,   67 }, 
    {    9,    8 }, {    8,   24 }, {    6,   38 }, {    4,   50 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   -3,   65 }, {   27,  -12 }, 
    {   15,    6 }, {    2,   29 }, {   28,  -14 }, {   22,   19 }, 
    {   52,  -82 }, {   32,  -49 }, {    4,   -5 }, {   56,  -82 }, 
    {    3,   55 }, {   24,  -22 }, {   34,  -30 }, {   17,   -7 }, 
    {   17,    5 }, {   17,   24 }, {   27,  -16 }, {   35,  -35 }, 
    {   22,  -14 }, {   42,  -42 }, {   -1,   61 }, {   40,  -38 }, 
    {   36,  -31 }, {    3,   17 }, {   10,   20 }, {   10,   41 }, 
    {   45,  -46 }, {   37,  -33 }, {   30,  -20 }, {   31,  -12 }, 
    {   -4,   68 }, {   37,  -29 }, {   30,  -12 }, {  -11,   52 }, 
    {    3,   37 }, {    8,   45 }, {   53,  -65 }, {   24,   -2 }, 
    {   26,   -6 }, {   13,   24 }, {   -4,   70 }, {   26,   -3 }, 
    {   -5,   55 }, {  -26,   84 }, {   -5,   61 }, {   -8,   75 }, 
    {   26,  -22 }, {   22,    2 }, {   15,   19 }, {   -7,   60 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   -7,   75 }, {   12,   17 }, 
    {    8,   21 }, {    2,   30 }, {   26,  -10 }, { -171,  413 }, 
    {  347, -693 }, {    0,    0 }, {    0,   64 }, {    0,   64 }, 
    {   -2,   63 }, {   30,  -16 }, {   24,   -6 }, {   21,    0 }, 
    {   17,   13 }, {    5,   47 }, {   19,    2 }, {   53,  -60 }, 
    {   37,  -36 }, {   39,  -29 }, {    1,   59 }, {   14,   18 }, 
    {   20,    6 }, {   18,   10 }, {   13,   24 }, {   16,   30 }, 
    {   42,  -33 }, {   38,  -31 }, {   38,  -28 }, {   26,    0 }, 
    {    1,   59 }, {   10,   33 }, {   15,   20 }, {    6,   37 }, 
    {    8,   35 }, {   10,   41 }, {   34,  -12 }, {  -16,   73 }, 
    {   -1,   43 }, {   12,   33 }, {  -13,   86 }, {   19,   16 }, 
    {    2,   45 }, {   -4,   56 }, {    1,   50 }, {    0,   61 }, 
    {   15,   17 }, {    7,   31 }, {  -22,   78 }, {   10,   29 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  }
};

// initial probability for coefficient level
static const Short
INIT_TCOEFF_LEVEL[3][70][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   11,   18 }, {    6,   32 }, 
    {   -4,   49 }, {   18,   16 }, {  -17,  102 }, {   38,  -29 }, 
    {   12,   13 }, {   41,  -33 }, {   32,  -11 }, {    0,   60 }, 
    {   14,   18 }, {   14,   27 }, {   18,   25 }, {   17,   29 }, 
    {   -1,   74 }, {    6,   34 }, {   11,   31 }, {   11,   35 }, 
    {   11,   36 }, {   -2,   72 }, {    4,   40 }, {    3,   50 }, 
    {    0,   61 }, {   -1,   66 }, {   -8,   88 }, {   -1,   55 }, 
    {    9,   39 }, {    6,   51 }, {  -31,  114 }, {    1,   69 }, 
    {   -6,   60 }, {   -6,   68 }, {   -6,   71 }, {   -5,   74 }, 
    {  -12,   93 }, {   -7,   58 }, {   -1,   61 }, {  -14,   88 }, 
    {    1,   63 }, {    1,   69 }, {  -14,   77 }, {  -11,   80 }, 
    {  -11,   87 }, {  -11,   89 }, {  -30,  128 }, {   -6,   58 }, 
    {   -9,   73 }, {   -7,   77 }, {  -25,  109 }, {  -26,  112 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {  -11,   61 }, {    0,   47 }, 
    {    4,   42 }, {   -9,   67 }, {   -8,   84 }, {   17,  -17 }, 
    {   69, -101 }, {   14,   11 }, {   72,  -81 }, {    4,   57 }, 
    {   -9,   62 }, {   -2,   54 }, {    2,   52 }, {    2,   54 }, 
    {   -8,   83 }, {   23,  -10 }, {   26,   -2 }, {    1,   49 }, 
    {   17,   23 }, {   12,   42 }, {  -15,   71 }, {   -1,   57 }, 
    {   -2,   62 }, {  -17,   92 }, {   -5,   78 }, {   26,   -7 }, 
    {   15,   26 }, {    3,   52 }, {    6,   51 }, {    4,   59 }, 
    {  -21,   80 }, {  -10,   73 }, {  -13,   83 }, {   -9,   79 }, 
    {  -12,   89 }, {   16,   18 }, {    4,   48 }, {    0,   60 }, 
    {    3,   55 }, {  -15,   92 }, {  -22,   81 }, {   -7,   70 }, 
    {  -12,   82 }, {  -12,   87 }, {  -21,  105 }, {   10,   28 }, 
    {   -1,   57 }, {   -5,   70 }, {  -10,   80 }, {  -49,  147 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {   -7,   51 }, {   -3,   48 }, 
    {  -18,   80 }, {  -22,   91 }, {  -17,   98 }, {   27,  -38 }, 
    {    0,   72 }, { -411,  898 }, {    0,   64 }, {    0,   64 }, 
    {    8,   23 }, {   14,   20 }, {   14,   24 }, {   12,   30 }, 
    {   -8,   82 }, {   23,   -5 }, {   10,   28 }, {   26,    3 }, 
    {   18,   20 }, {   23,   21 }, {   -7,   59 }, {   -3,   60 }, 
    {   -4,   66 }, {   -5,   70 }, {   -4,   76 }, {   23,    0 }, 
    {   12,   32 }, {    9,   42 }, {    4,   53 }, {  -11,   86 }, 
    {   -4,   52 }, {   -5,   63 }, {  -21,   97 }, {  -24,  104 }, 
    {  -23,  106 }, {    3,   40 }, {    7,   42 }, {   19,   20 }, 
    {   -2,   68 }, {  -35,  125 }, {   -3,   50 }, {   -4,   62 }, 
    {  -12,   80 }, {   -7,   74 }, {  -15,   94 }, {   20,    7 }, 
    {    4,   45 }, {  -30,  109 }, {  -41,  129 }, {  -23,  101 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  }
};
#endif

// initial probability for motion vector predictor index
static const Short
INIT_MVP_IDX[3][NUM_MVP_IDX_CTX][2] =
{
  {
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }
  }
};

#ifdef DCM_PBIC
// initial probability for intensity compensation predictor index
static const Short INIT_ICP_IDX[3][NUM_ICP_IDX_CTX][2] = 
{
  // for I slice
  {{0,64},{0,64}},
  // for P slice
  {{0,64},{0,64}},
  // for B slice
  {{0,64},{0,64}}
};

// initial probability for zero flag of zero tree
static const Short INIT_ZTree_MV0[3][NUM_ZTREE_MV0_CTX][2] = 
{
  // for I slice
  {{0,64}},
  // for P slice
  {{0,64}},
  // for B slice
  {{0,64}}
};

// initial probability for zero tree used to code MV and IC parameters in Uni-Pred case
static const Short INIT_ZTree_MV1[3][NUM_ZTREE_MV1_CTX][2] = 
{
  // for I slice
  {{0,64},{0,64},{0,64},{0,64},{0,64},{0,64}},
  // for P slice
  {{0,36},{0,117},{0,64},{0,59},{0,51},{0,80}},
  // for B slice
  {{0,36},{0,117},{0,64},{0,59},{0,51},{0,80}}
};

// initial probability for zero tree used to code MV and IC parameters in Bi-Pred case
static const Short INIT_ZTree_MV2[3][NUM_ZTREE_MV2_CTX][2] = 
{
  // for I slice
  {{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64}},
  // for P slice
  {{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64},{0,64}},
  // for B slice
  {{0,30},{0,123},{0,68},{0,51},{0,63},{0,64},{0,64},{0,68},{0,43},{0,59},{0,43},{0,63}}
};
#endif

// initial probability for ROT index
static const Short
  INIT_ROT_IDX[3][3][2] ={
    {
      {    0,   55 }, {    1,   59 }, {    0,   65 }
    },
    {
      {  -10,   79 }, {   -3,   65 }, {  -20,   99 }
    },
    {
      {  -16,   94 }, {   -5,   68 }, {  -17,   92 }
    }
};

// initial probability for CIP index
static const Short
INIT_CIP_IDX[3][NUM_CIP_FLAG_CTX][2] =
{
  {
    {  -12,   46 }, {  -13,   60 }, {  -21,   80 }
  },
  {
    {  -14,   67 }, {  -25,   93 }, {  -32,  106 }
  },
  {
    {   -9,   68 }, {   -8,   71 }, {  -17,   79 }
  }
};

// initial probability for ALF flag
static const Short
INIT_ALF_FLAG[3][NUM_ALF_FLAG_CTX][2] =
{
  {
    {   50,  -48 }
  },
  {
    {   27,  -20 }
  },
  {
    {  -12,   68 }
  }
};

#if HHI_ALF
// initial probability for ALF flag
static const Short
INIT_ALF_SPLITFLAG[3][NUM_ALF_SPLITFLAG_CTX][2] =
{
  {
    {  -13,   77 }
  },
  {
    {  -23,   87 }
  },
  {
    {  -34,   98 }
  }
};
#endif

// initial probability for ALF side information (unsigned)
static const Short
INIT_ALF_UVLC[3][NUM_ALF_UVLC_CTX][2] =
{
  {
    {    1,   66 }, {   -3,   77 }
  },
  {
    {   -5,   75 }, {  -14,   94 }
  },
  {
    {   -5,   72 }, {  -30,  122 }
  }
};

// initial probability for ALF side information (signed)
static const Short
INIT_ALF_SVLC[3][NUM_ALF_SVLC_CTX][2] =
{
  {
    {   11,   57 }, {   -1,   62 }, {    0,   64 }
  },
  {
    {    6,   66 }, {   -1,   64 }, {    0,   64 }
  },
  {
    {    1,   73 }, {    2,   61 }, {    0,   64 }
  }
};

#if HHI_RQT
static const Short
INIT_TRANS_SUBDIV_FLAG[3][NUM_TRANS_SUBDIV_FLAG_CTX][2] =
{
  {
    {    0,    0 }, {   12,   12 }, {   22,    4 }, {   -2,   49 }, 
    {    4,   46 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   13 }, {  -28,   89 }, {  -30,   99 }, {  -34,  106 }, 
    {  -19,   76 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  },
  {
    {  -11,   38 }, {  -31,   88 }, {  -42,  118 }, {  -47,  130 }, 
    {  -21,   73 }, {    0,   64 }, {    0,   64 }, {    0,   64 }, 
    {    0,   64 }, {    0,   64 }
  }
};
#endif

// initial probability for transform index
static const Short
INIT_TRANS_IDX[3][4][2] =
{
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }
  },
  {
    {    0,   64 }, {    0,   64 }, {    0,   64 }, {    0,   64 }
  }
};

#endif

>>>>>>> upstream/master
