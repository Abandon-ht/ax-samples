/**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <assert.h>
#include <pthread.h>
#include <unistd.h>
#include <getopt.h>
#include <tinyalsa/pcm.h>
#include <samplerate.h>
#include "ax_base_type.h"
#include "ax_aenc_api.h"
#include "ax_adec_api.h"
#include "ax_audio_process.h"
#include "ax_sys_api.h"
#include "wave_parser.h"
#include "ax_ai_api.h"
#include "ax_ao_api.h"
#include "ax_global_type.h"
#include "ax_acodec_api.h"
#include "ax_aac.h"

#ifndef CHIP_TYPE_AX620Q
#include "ax_opus.h"
#endif

#define FILE_NAME_SIZE      128
#define STRING_LEN      128
#define RESAMPLE_NUM        16*1024
//#define RESAMPLE_PERF
#include "wave_parser.c.h"
static unsigned int gCardNum = 0;
static unsigned int gDeviceNum = 0;
static unsigned int gChannels = 2;
static unsigned int gRate = 16000;
static unsigned int gEncodeRate = 16000;
static unsigned int gBits = 16;
static const char *gEncoderType = "g711a";
static AX_S32 gWriteFrames = 0;
static AX_S32 gLoopExit = 0;
static AX_S32 gVqeSampleRate = 16000;
static AX_S32 gResRate = 32000;

static unsigned int gPeriodSize = 160;
static unsigned int gPeriodCount = 8;
static int gIsWave = 1;
static AX_AEC_CONFIG_T gAecCfg = {
    .enAecMode = AX_AEC_MODE_DISABLE,
};
static AX_NS_CONFIG_T gNsCfg = {
    .bNsEnable = AX_FALSE,
    .enAggressivenessLevel = static_cast<AX_AGGRESSIVENESS_LEVEL_E>(2),
};
static AX_AGC_CONFIG_T gAgcCfg = {
    .bAgcEnable = AX_FALSE,
    .enAgcMode = AX_AGC_MODE_FIXED_DIGITAL,
    .s16TargetLevel = -3,
    .s16Gain = 9,
};

static AX_ACODEC_EQ_ATTR_T gEqCfg = {
    .s32GainDb = {-10,-3,3,5,10},
    .s32Samplerate = 16000,
    .bEnable = AX_FALSE,
};

static AX_ACODEC_FREQ_ATTR_T gHpfCfg = {
    .s32GainDb = -3,
    .s32Samplerate = 16000,
    .s32Freq = 200,
    .bEnable = AX_FALSE,
};
static AX_ACODEC_FREQ_ATTR_T gLpfCfg = {
    .s32GainDb = 0,
    .s32Samplerate = 16000,
    .s32Freq = 3000,
    .bEnable = AX_FALSE,
};

static unsigned int gAencChannels = 2;
static AX_AI_LAYOUT_MODE_E gLayoutMode = AX_AI_MIC_MIC;
static int gResample = 0;
static int gConverter = SRC_SINC_FASTEST;
static AX_AAC_TYPE_E gAacType = AX_AAC_TYPE_AAC_LC;
static AX_AAC_TRANS_TYPE_E gTransType = AX_AAC_TRANS_TYPE_ADTS;
static AX_F64 gVqeVolume = 1.0;
static AX_S32 gGetNumber = -1;
static AX_S32 gLoopNumber = 1;
static const char *gInputFile = NULL;
static const char *gOutputFile = NULL;
static const char *gLengthFile = NULL;
static const char *gAscFile = NULL;
static AX_S32 gSaveFile = 0;
static AX_S32 gCtrl = 0;
static AX_S32 gInstant = 0;
static AX_S32 gInsertSilence = 0;
static AX_S32 gSimDrop = 0;
static AX_S32 gDbDetection = 0;
static AX_S32 gGetNoise = 0;
static AX_S32 gMix = 0;
static const char *gMixFile = NULL;
static AX_S32 gAsyncTest = 0;
static const char *gAsyncTestName = NULL;
static AX_S32 gAsyncTestNumber = 10;

bool stop = false;

int BitsToFormat(unsigned int bits, AX_AUDIO_BIT_WIDTH_E* format)
{
    switch (bits) {
    case 32:
        *format = AX_AUDIO_BIT_WIDTH_32;
        break;
    case 24:
        *format = AX_AUDIO_BIT_WIDTH_24;
        break;
    case 16:
        *format = AX_AUDIO_BIT_WIDTH_16;
        break;
    default:
        fprintf(stderr, "%u bits is not supported.\n", bits);
        return -1;
    }

    return 0;
}

int IsUpTalkVqeEnabled(const AX_AP_UPTALKVQE_ATTR_T *pstVqeAttr)
{
    int ret = ((pstVqeAttr->stAecCfg.enAecMode != AX_AEC_MODE_DISABLE) ||
        (pstVqeAttr->stNsCfg.bNsEnable != AX_FALSE) ||
        (pstVqeAttr->stAgcCfg.bAgcEnable != AX_FALSE));

    return ret;
}


int IsDnVqeEnabled(const AX_AP_DNVQE_ATTR_T *pstVqeAttr)
{
    int ret = ((pstVqeAttr->stNsCfg.bNsEnable != AX_FALSE) ||
        (pstVqeAttr->stAgcCfg.bAgcEnable != AX_FALSE));

    return ret;
}

typedef struct axSAMPLE_AI_CTRL_ARGS_S {
    AI_CARD aiCardId;
    AI_DEV aiDevId;
} SAMPLE_AI_CTRL_ARGS_S;

static void *AiCtrlThread(void *arg)
{
    AX_S32 ret = AX_SUCCESS;
    SAMPLE_AI_CTRL_ARGS_S *aiCtrlArgs = (SAMPLE_AI_CTRL_ARGS_S *)arg;
    AX_CHAR key, key0, key1;
    int index = 0;

    while (!gLoopExit) {
        printf("please enter[save(s), unsave(a), VqeVolume=1.0(1), VqeVolume=5.0(5), EnableResample(r), DisableResample(e), ToggleResample(t), quit(q)]:\n");
        key0 = getchar();
        key1 = getchar();
        key = (key0 == '\n') ? key1 : key0;

        switch (key) {
        case 's':
        case 'S': {
            AX_AUDIO_SAVE_FILE_INFO_T stSaveFileInfo;
            stSaveFileInfo.bCfg = AX_TRUE;
            strncpy(stSaveFileInfo.aFilePath, "./", AX_MAX_AUDIO_FILE_PATH_LEN);
            strncpy(stSaveFileInfo.aFileName, "default", AX_MAX_AUDIO_FILE_NAME_LEN);
            stSaveFileInfo.u32FileSize = 1024;
            ret = AX_AI_SaveFile(aiCtrlArgs->aiCardId, aiCtrlArgs->aiDevId, &stSaveFileInfo);
            printf("AX_AI_SaveFile, ret: %x\n", ret);
            break;
        }
        case 'a':
        case 'A': {
            AX_AUDIO_SAVE_FILE_INFO_T stSaveFileInfo;
            stSaveFileInfo.bCfg = AX_FALSE;
            strncpy(stSaveFileInfo.aFilePath, "./", AX_MAX_AUDIO_FILE_PATH_LEN);
            strncpy(stSaveFileInfo.aFileName, "default", AX_MAX_AUDIO_FILE_NAME_LEN);
            stSaveFileInfo.u32FileSize = 1024;
            ret = AX_AI_SaveFile(aiCtrlArgs->aiCardId, aiCtrlArgs->aiDevId, &stSaveFileInfo);
            printf("AX_AI_SaveFile, ret: %x\n", ret);
            break;
        }
        case '1': {
            ret = AX_AI_SetVqeVolume(aiCtrlArgs->aiCardId, aiCtrlArgs->aiDevId, 1.0);
            printf("AX_AI_SetVqeVolume, ret: %x\n", ret);
            break;
        }
        case '5': {
            ret = AX_AI_SetVqeVolume(aiCtrlArgs->aiCardId, aiCtrlArgs->aiDevId, 5.0);
            printf("AX_AI_SetVqeVolume, ret: %x\n", ret);
            break;
        }
        case 'r':
        case 'R': {
            AX_AUDIO_SAMPLE_RATE_E enOutSampleRate = (AX_AUDIO_SAMPLE_RATE_E)gResRate;
            ret = AX_AI_EnableResample(aiCtrlArgs->aiCardId, aiCtrlArgs->aiDevId, enOutSampleRate);
            printf("AX_AI_EnableResample, ret: %x\n", ret);
            break;
        }
        case 'e':
        case 'E': {
            ret = AX_AI_DisableResample(aiCtrlArgs->aiCardId, aiCtrlArgs->aiDevId);
            printf("AX_AI_DisableResample, ret: %x\n", ret);
            break;
        }
        case 't':
        case 'T': {
            AX_AUDIO_SAMPLE_RATE_E enOutSampleRateArray[3] = {AX_AUDIO_SAMPLE_RATE_8000, AX_AUDIO_SAMPLE_RATE_32000, AX_AUDIO_SAMPLE_RATE_48000};
            ret = AX_AI_DisableResample(aiCtrlArgs->aiCardId, aiCtrlArgs->aiDevId);
            printf("AX_AI_DisableResample, ret: %x\n", ret);
            ret = AX_AI_EnableResample(aiCtrlArgs->aiCardId, aiCtrlArgs->aiDevId, enOutSampleRateArray[index]);
            printf("AX_AI_EnableResample, ret: %x\n", ret);
            index++;
            index %= 3;
            break;
        }
        case 'q':
        case 'Q': {
            gLoopExit = 1;
            break;
        }
        }
    }

    printf("AiCtrlThread exit\n");
    return NULL;
}

typedef struct axSAMPLE_AI_AED_RECV_ARGS_S {
    AI_CARD aiCardId;
    AI_DEV aiDevId;
} SAMPLE_AI_AED_RECV_ARGS_S;

static void *AiAedRecvThread(void *arg)
{
    AX_S32 ret = AX_SUCCESS;
    SAMPLE_AI_AED_RECV_ARGS_S *aiAedRecvArgs = (SAMPLE_AI_AED_RECV_ARGS_S *)arg;

    AX_S32 getNumber = 0;
    while (!gLoopExit) {
        AX_AED_RESULT_INFO stAedResultInfo;
        ret = AX_AI_GetAedResult(aiAedRecvArgs->aiCardId, aiAedRecvArgs->aiDevId, &stAedResultInfo);
        if (ret) {
            printf("AX_AI_GetAedResult error: %x\n", ret);
            break;
        }

        getNumber++;
        printf("stAedResultInfo.s32Db: %d\n", stAedResultInfo.s32Db);

        if (((gGetNumber > 0) && (getNumber >= gGetNumber)) || gLoopExit) {
            printf("getNumber: %d\n", getNumber);
            break;
        }
    }

    printf("AiAedRecvThread exit\n");
    return NULL;
}

typedef struct axSAMPLE_AO_CTRL_ARGS_S {
    AO_CARD aoCardId;
    AO_DEV aoDevId;
} SAMPLE_AO_CTRL_ARGS_S;

typedef struct axSAMPLE_AENC_ARGS_S {
    AENC_CHN aeChn;
    AX_PAYLOAD_TYPE_E payloadType;
    const char* fileExt;
} SAMPLE_AENC_ARGS_S;

static void SigInt(int sigNo)
{
    printf("Catch signal %d\n", sigNo);
    gLoopExit = 1;
}

static void PrintHelp()
{
    printf("usage: sample_audio      <command> <args>\n");
    printf("commands:\n");
    printf("ai:                      ai get data.\n");
    printf("ao:                      ao play data.\n");
    printf("ai_aenc:                 aenc link mode.\n");
    printf("adec_ao:                 decode link mode.\n");
    printf("args:\n");
    printf("  -D:                    card number.                (support 0), default: 0\n");
    printf("  -d:                    device number.              (support 0,1,2,3), default: 0\n");
    printf("  -c:                    channels.                   (support 2,4), default: 2\n");
    printf("  -r:                    rate.                       (support 8000~48000), default: 48000\n");
    printf("  -b:                    bits.                       (support 16,32), default: 16\n");
    printf("  -p:                    period size.                (support 80~1024), default: 1024\n");
    printf("  -v:                     is wave file.               (support 0,1), default: 1\n");
    printf("  -e:                     encoder type.               (support g711a, g711u, aac, lpcm, g726, opus), default: g711a\n");
    printf("  -w:                     write audio frame to file.  (support 0,1), default: 0\n");
    printf("  -G:                     get number.                 (support int), default: -1\n");
    printf("  -L:                     loop number.                (support int), default: 1\n");
    printf("  -i:                     input file.                 (support char*), default: NULL\n");
    printf("  -o:                     output file.                (support char*), default: NULL\n");
    printf("  --aec-mode:             aec mode.                   (support 0,1,2), default: 0\n");
    printf("  --sup-level:            Suppression Level.          (support 0,1,2), default: 0\n");
    printf("  --routing-mode:         routing mode.               (support 0,1,2,3,4), default: 0\n");
    printf("  --aenc-chns:            encode channels.            (support 1,2), default: 2\n");
    printf("  --layout:               layout mode.                (support 0,1,2), default: 0\n");
    printf("  --ns:                   ns enable.                  (support 0,1), default: 0\n");
    printf("  --ag-level:             aggressiveness level.       (support 0,1,2,3), default: 2\n");
    printf("  --agc:                  agc enable.                 (support 0,1), default: 0\n");
    printf("  --target-level:         target level.               (support -31~0), default: -3\n");
    printf("  --gain:                 compression gain.           (support 0~90), default: 9\n");
    printf("  --resample:             resample enable.            (support 0,1), default: 0\n");
    printf("  --resrate:              resample rate.              (support 8000~48000), default: 16000\n");
    printf("  --vqe-volume:           vqe volume.                 (support 0~10.0), default: 1.0\n");
    printf("  --converter:            converter type.             (support 0~4), default: 2\n");
    printf("  --aac-type:             aac type.                   (support 2,23,39), default: 2\n");
    printf("  --trans-type:           trans type.                 (support 0,2), default: 2\n");
    printf("  --asc-file:             asc file.                   (support char*), default: NULL\n");
    printf("  --length-file:          length file.                (support char*), default: NULL\n");
    printf("  --save-file:            save file.                  (support 0,1), default: 0\n");
    printf("  --ctrl:                 ctrl enable.                (support 0,1), default: 0\n");
    printf("  --instant:              instant enable.             (support 0,1), default: 0\n");
    printf("  --period-count:         period count.               (support int), default: 4\n");
    printf("  --insert-silence:       insert silence enable.      (support int), default: 0\n");
    printf("  --sim-drop:             sim drop enable.            (support int), default: 0\n");
    printf("  --db-detection:         db detection enable.        (support int), default: 0\n");
    printf("  --mix:                  mix enable.                 (support int), default: 0\n");
    printf("  --mix-file:             mix file.                   (support char*), default: NULL\n");
    printf("  --async-test:           async test enable.          (support int), default: 0\n");
    printf("  --async-test-name:      async test name.            (support char*), default: NULL\n");
    printf("  --async-test-number:    async test number.          (support int), default: 10\n");
    printf("  --hpf:                  hpf enable.                 (support int), default: 0\n");
    printf("  --hpf-freq:             hpf frequency.              (support int), default: 200\n");
    printf("  --hpf-db:               hpf db.                     (support int), default: -3\n");
    printf("  --lpf:                  lpf enable.                 (support int), default: 0\n");
    printf("  --lpf-freq:             lpf frequency.              (support int), default: 3000\n");
    printf("  --lpf-db:               lpf db.                     (support int), default: 0\n");
    printf("  --eq:                   eq enable.                  (support int), default: 0\n");
}

enum LONG_OPTION {
    LONG_OPTION_AEC_MODE = 10000,
    LONG_OPTION_SUPPRESSION_LEVEL,
    LONG_OPTION_ROUTING_MODE,
    LONG_OPTION_AENC_CHANNELS,
    LONG_OPTION_LAYOUT_MODE,
    LONG_OPTION_NS_ENABLE,
    LONG_OPTION_AGGRESSIVENESS_LEVEL,
    LONG_OPTION_AGC_ENABLE,
    LONG_OPTION_TARGET_LEVEL,
    LONG_OPTION_GAIN,
    LONG_OPTION_RESAMPLE,
    LONG_OPTION_RESAMPLE_RATE,
    LONG_OPTION_VQE_VOLUME,
    LONG_OPTION_CONVERTER,
    LONG_OPTION_AAC_TYPE,
    LONG_OPTION_AAC_TRANS_TYPE,
    LONG_OPTION_ASC_FILE,
    LONG_OPTION_LENGTH_FILE,
    LONG_OPTION_SAVE_FILE,
    LONG_OPTION_CTRL,
    LONG_OPTION_INSTANT,
    LONG_OPTION_PERIOD_COUNT,
    LONG_OPTION_INSERT_SILENCE,
    LONG_OPTION_SIM_DROP,
    LONG_OPTION_DB_DETECTION,
    LONG_OPTION_GET_NOISE,
    LONG_OPTION_MIX,
    LONG_OPTION_MIX_FILE,
    LONG_OPTION_ASYNC_TEST,
    LONG_OPTION_ASYNC_TEST_NAME,
    LONG_OPTION_ASYNC_TEST_NUMBER,
    LONG_OPTION_HPF,
    LONG_OPTION_HPF_FRE,
    LONG_OPTION_HPF_DB,
    LONG_OPTION_LPF,
    LONG_OPTION_LPF_FRE,
    LONG_OPTION_LPF_DB,
    LONG_OPTION_EQ,
    LONG_OPTION_BUTT
};

static int AudioInput()
{
    int index = 0;
    while (!stop) {
        int ret = 0;
        unsigned int card = gCardNum;
        unsigned int device = gDeviceNum;
        AX_AUDIO_BIT_WIDTH_E format;
        unsigned int totalFrames = 0;
        FILE *output_file = NULL;

        if (BitsToFormat(gBits, &format))
            return -1;

        ret = AX_SYS_Init();
        if (AX_SUCCESS != ret) {
            printf("AX_SYS_Init failed! Error Code:0x%X\n", ret);
            return -1;
        }

        AX_POOL_CONFIG_T stPoolConfig;
        stPoolConfig.MetaSize = 8192;
        stPoolConfig.BlkSize = 7680;
        stPoolConfig.BlkCnt = 33;
        stPoolConfig.IsMergeMode = AX_FALSE;
        stPoolConfig.CacheMode = AX_POOL_CACHE_MODE_NONCACHE;
        strcpy((char *)stPoolConfig.PartitionName, "anonymous");
        AX_POOL PoolId = AX_POOL_CreatePool(&stPoolConfig);
        if (PoolId == AX_INVALID_POOLID) {
            printf("AX_POOL_CreatePool failed! PoolId:%d\n", PoolId);
            AX_SYS_Deinit();
        }

        AX_AUDIO_FRAME_T stFrame;
        char output_file_name[FILE_NAME_SIZE];
        if (gOutputFile) {
            strncpy(output_file_name, gOutputFile, FILE_NAME_SIZE-1);
        } else {
            snprintf(output_file_name, FILE_NAME_SIZE, "audio_%d.%s", index, gIsWave ? "wav" : "raw");
        }
        if (gWriteFrames) {
            output_file = fopen(output_file_name, "wb");
            assert(output_file != NULL);
            if (gIsWave) {
                LeaveWaveHeader(output_file);
            }
        }

        ret = AX_AI_Init();
        if (ret) {
            printf("AX_AI_Init failed! Error Code:0x%X\n", ret);
            AX_POOL_DestroyPool(PoolId);
        }

        AX_AI_ATTR_T stAttr;
        stAttr.enBitwidth = format;
        stAttr.enLinkMode = AX_UNLINK_MODE;
        stAttr.enSamplerate = (AX_AUDIO_SAMPLE_RATE_E)gRate;
        stAttr.enLayoutMode = gLayoutMode;
        stAttr.U32Depth = 30;
        stAttr.u32PeriodSize = gPeriodSize;
        stAttr.u32PeriodCount = gPeriodCount;
        stAttr.u32ChnCnt = gChannels;
        ret = AX_AI_SetPubAttr(card,device,&stAttr);
        if(ret){
            printf("AX_AI_SetPubAttr failed! ret = %x\n", ret);
            AX_AI_DeInit();
        }

        ret = AX_AI_AttachPool(card,device,PoolId);
        if(ret){
            printf("AX_AI_AttachPool failed! ret = %x\n", ret);
            AX_AI_DeInit();
        }

        unsigned int outRate = gRate;
        AX_AP_UPTALKVQE_ATTR_T stVqeAttr;
        memset(&stVqeAttr, 0, sizeof(stVqeAttr));
        stVqeAttr.s32SampleRate = gVqeSampleRate;
        stVqeAttr.u32FrameSamples = 160;
        memcpy(&stVqeAttr.stNsCfg, &gNsCfg, sizeof(AX_NS_CONFIG_T));
        memcpy(&stVqeAttr.stAgcCfg, &gAgcCfg, sizeof(AX_AGC_CONFIG_T));
        memcpy(&stVqeAttr.stAecCfg, &gAecCfg, sizeof(AX_AEC_CONFIG_T));
        if (IsUpTalkVqeEnabled(&stVqeAttr)) {
            ret = AX_AI_SetUpTalkVqeAttr(card, device, &stVqeAttr);
            if(ret){
                printf("AX_AI_SetUpTalkVqeAttr failed! ret = %x\n",ret);
                AX_AI_DetachPool(card, device);
            }
            outRate = gVqeSampleRate;
        }

        if (gHpfCfg.bEnable) {
            AX_ACODEC_FREQ_ATTR_T stHpfAttr;
            stHpfAttr.s32Freq = gHpfCfg.s32Freq;
            stHpfAttr.s32GainDb = gHpfCfg.s32GainDb;
            stHpfAttr.s32Samplerate = gRate;
            ret = AX_ACODEC_RxHpfSetAttr(card, &stHpfAttr);
            if(ret){
                printf("AX_ACODEC_RxHpfSetAttr failed! ret = %x\n", ret);
                AX_AI_DetachPool(card, device);
            }
            ret = AX_ACODEC_RxHpfEnable(card);
            if(ret){
                printf("AX_ACODEC_RxHpfEnable failed! ret = %x\n", ret);
                AX_AI_DetachPool(card, device);
            }
        }
        if (gLpfCfg.bEnable) {
            AX_ACODEC_FREQ_ATTR_T stLpfAttr;
            stLpfAttr.s32Freq = gLpfCfg.s32Freq;
            stLpfAttr.s32GainDb = gLpfCfg.s32GainDb;
            stLpfAttr.s32Samplerate = gRate;
            ret = AX_ACODEC_RxLpfSetAttr(card, &stLpfAttr);
            if(ret){
                printf("AX_ACODEC_RxLpfSetAttr failed! ret = %x\n", ret);
                if (gHpfCfg.bEnable) {
                    ret = AX_ACODEC_RxHpfDisable(card);
                    if(ret){
                        printf("AX_ACODEC_RxHpfDisable failed! ret= %x\n",ret);
                    }
                }
            }
            ret = AX_ACODEC_RxLpfEnable(card);
            if(ret){
                printf("AX_ACODEC_RxLpfEnable failed! ret = %x\n", ret);
                if (gHpfCfg.bEnable) {
                    ret = AX_ACODEC_RxHpfDisable(card);
                    if(ret){
                        printf("AX_ACODEC_RxHpfDisable failed! ret= %x\n",ret);
                    }
                }
            }
        }
        if (gEqCfg.bEnable) {
            AX_ACODEC_EQ_ATTR_T stEqAttr;
            memcpy(&stEqAttr.s32GainDb, &gEqCfg.s32GainDb, sizeof(stEqAttr.s32GainDb));
            stEqAttr.s32Samplerate = gRate;
            ret = AX_ACODEC_RxEqSetAttr(card, &stEqAttr);
            if(ret){
                printf("AX_ACODEC_RxEqSetAttr failed! ret = %x\n", ret);
                if (gLpfCfg.bEnable) {
                    ret = AX_ACODEC_RxLpfDisable(card);
                    if(ret){
                        printf("AX_ACODEC_RxLpfDisable failed! ret= %x\n",ret);
                    }
                }
            }
            ret = AX_ACODEC_RxEqEnable(card);
            if(ret){
                printf("AX_ACODEC_RxEqEnable failed! ret = %x\n", ret);
                if (gLpfCfg.bEnable) {
                    ret = AX_ACODEC_RxLpfDisable(card);
                    if(ret){
                        printf("AX_ACODEC_RxLpfDisable failed! ret= %x\n",ret);
                    }
                }
            }
        }

        ret = AX_AI_EnableDev(card,device);
        if (ret){
            printf("AX_AI_EnableDev failed! ret = %x \n",ret);
            if (gEqCfg.bEnable) {
                ret = AX_ACODEC_RxEqDisable(card);
                if(ret){
                    printf("AX_ACODEC_RxEqDisable failed! ret= %x\n",ret);
                }
            }
        }

        if (gResample) {
            AX_AUDIO_SAMPLE_RATE_E enOutSampleRate = (AX_AUDIO_SAMPLE_RATE_E)gResRate;
            ret = AX_AI_EnableResample(card, device, enOutSampleRate);
            if(ret){
                printf("AX_AI_EnableResample failed! ret = %x,\n",ret);
                AX_AI_DisableDev(card, device);
            }
            outRate = gResRate;
        }

        ret = AX_AI_SetVqeVolume(card, device, gVqeVolume);
        if(ret){
            printf("AX_AI_SetVqeVolume failed! ret = %x\n", ret);
            AX_AI_DisableDev(card, device);
        }

        if (gSaveFile) {
            AX_AUDIO_SAVE_FILE_INFO_T stSaveFileInfo;
            stSaveFileInfo.bCfg = AX_TRUE;
            strncpy(stSaveFileInfo.aFilePath, "./", AX_MAX_AUDIO_FILE_PATH_LEN);
            strncpy(stSaveFileInfo.aFileName, "default", AX_MAX_AUDIO_FILE_NAME_LEN);
            stSaveFileInfo.u32FileSize = 1024;
            ret = AX_AI_SaveFile(card, device, &stSaveFileInfo);
            if(ret){
                printf("AX_AI_SaveFile failed! ret = %x\n", ret);
                AX_AI_DisableDev(card, device);
            }
        }

        SAMPLE_AI_AED_RECV_ARGS_S aiAedRecvArgs;
        aiAedRecvArgs.aiCardId = card;
        aiAedRecvArgs.aiDevId = device;
        pthread_t aedRecvTid;
        if (gDbDetection) {
            AX_AED_ATTR_T stAedAttr;
            stAedAttr.bDbDetection = (AX_BOOL)gDbDetection;
            ret = AX_AI_SetAedAttr(card, device, &stAedAttr);
            if(ret){
                printf("AX_AI_SetAedAttr failed! ret = %x\n", ret);
                AX_AI_DisableDev(card, device);
            }

            ret = AX_AI_EnableAed(card, device);
            if(ret){
                printf("AX_AI_EnableAed failed! ret = %x\n", ret);
                AX_AI_DisableDev(card, device);
            }

            pthread_create(&aedRecvTid, NULL, AiAedRecvThread, (void *)&aiAedRecvArgs);
        }

        SAMPLE_AI_CTRL_ARGS_S aiCtrlArgs;
        aiCtrlArgs.aiCardId = card;
        aiCtrlArgs.aiDevId = device;
        pthread_t ctrlTid;
        if (gCtrl) {
            pthread_create(&ctrlTid, NULL, AiCtrlThread, (void *)&aiCtrlArgs);
        }

        AX_S32 getNumber = 0;
        while (1) {
            ret = AX_AI_GetFrame(card, device, &stFrame, -1);
            if (ret != AX_SUCCESS) {
                printf("AX_AI_GetFrame error, ret: %x\n",ret);
                break;
            }
            getNumber++;
            if (gWriteFrames)
                fwrite(stFrame.u64VirAddr, 2, stFrame.u32Len/2, output_file);

            totalFrames += stFrame.u32Len/2;
            ret = AX_AI_ReleaseFrame(card,device,&stFrame);
            if (ret) {
                printf("AX_AI_ReleaseFrame failed! ret=%x\n",ret);
            }
            printf("Now totalFrames = %d\n", totalFrames);
            if (totalFrames == 60000) {
                printf("Stop totalFrames = %d\n", totalFrames);
                // gLoopExit = 1;
                break;
            }
            if (((gGetNumber > 0) && (getNumber >= gGetNumber)) || gLoopExit) {
                printf("getNumber: %d\n", getNumber);
                return 0;
            }
        }
        if (gWriteFrames) {
            if (gIsWave) {
                if ((gChannels == 2) && (stAttr.enLayoutMode != AX_AI_MIC_MIC) && (stAttr.enLayoutMode != AX_AI_DOORBELL)) {
                    WriteWaveHeader(output_file, 1, outRate, gBits, totalFrames);
                } else {
                    WriteWaveHeader(output_file, gChannels, outRate, gBits, totalFrames/2);
                }
            }

            if(output_file) {
                fclose(output_file);
                index++;
            }
        }

        printf("totalFrames: %u\n", totalFrames);
        printf("ai success.\n");
        printf("Index = %d\n", index);
        if (index == 10)
            index = 0;

        if (gCtrl) {
            pthread_join(ctrlTid, NULL);
        }

        if (gDbDetection) {
            pthread_join(aedRecvTid, NULL);
        }

        if (gDbDetection) {
            ret = AX_AI_DisableAed(card, device);
            if(ret){
                printf("AX_AI_DisableAed failed! ret= %x\n",ret);
                AX_SYS_Deinit();
            }
        }

    }
}
int main(int argc, char *argv[])
{
    extern int optind;
    AX_S32 c;
    AX_S32 isExit = 0;
    signal(SIGINT, SigInt);

    while (1) {
        int option_index = 0;
        static struct option long_options[] = {
            {"aec-mode",            required_argument,  0, LONG_OPTION_AEC_MODE},
            {"sup-level",           required_argument,  0, LONG_OPTION_SUPPRESSION_LEVEL },
            {"routing-mode",        required_argument,  0, LONG_OPTION_ROUTING_MODE },
            {"aenc-chns",           required_argument,  0, LONG_OPTION_AENC_CHANNELS },
            {"layout",              required_argument,  0, LONG_OPTION_LAYOUT_MODE },
            {"ns",                  required_argument,  0, LONG_OPTION_NS_ENABLE },
            {"ag-level",            required_argument,  0, LONG_OPTION_AGGRESSIVENESS_LEVEL },
            {"agc",                 required_argument,  0, LONG_OPTION_AGC_ENABLE },
            {"target-level",        required_argument,  0, LONG_OPTION_TARGET_LEVEL },
            {"gain",                required_argument,  0, LONG_OPTION_GAIN },
            {"resample",            required_argument,  0, LONG_OPTION_RESAMPLE },
            {"resrate",              required_argument,  0, LONG_OPTION_RESAMPLE_RATE },
            {"vqe-volume",          required_argument,  0, LONG_OPTION_VQE_VOLUME },
            {"converter",           required_argument,  0, LONG_OPTION_CONVERTER },
            {"aac-type",            required_argument,  0, LONG_OPTION_AAC_TYPE },
            {"trans-type",          required_argument,  0, LONG_OPTION_AAC_TRANS_TYPE },
            {"asc-file",            required_argument,  0, LONG_OPTION_ASC_FILE },
            {"length-file",         required_argument,  0, LONG_OPTION_LENGTH_FILE },
            {"save-file",           required_argument,  0, LONG_OPTION_SAVE_FILE },
            {"ctrl",                required_argument,  0, LONG_OPTION_CTRL },
            {"instant",             required_argument,  0, LONG_OPTION_INSTANT },
            {"period-count",        required_argument,  0, LONG_OPTION_PERIOD_COUNT },
            {"insert-silence",      required_argument,  0, LONG_OPTION_INSERT_SILENCE},
            {"sim-drop",            required_argument,  0, LONG_OPTION_SIM_DROP},
            {"db-detection",        required_argument,  0, LONG_OPTION_DB_DETECTION},
            {"get-noise",           required_argument,  0, LONG_OPTION_GET_NOISE},
            {"mix",                 required_argument,  0, LONG_OPTION_MIX},
            {"mix-file",            required_argument,  0, LONG_OPTION_MIX_FILE},
            {"async-test",          required_argument,  0, LONG_OPTION_ASYNC_TEST},
            {"async-test-name",     required_argument,  0, LONG_OPTION_ASYNC_TEST_NAME},
            {"async-test-number",   required_argument,  0, LONG_OPTION_ASYNC_TEST_NUMBER},
            {"hpf",                 required_argument,  0, LONG_OPTION_HPF},
            {"hpf-freq",            required_argument,  0, LONG_OPTION_HPF_FRE},
            {"hpf-db",              required_argument,  0, LONG_OPTION_HPF_DB},
            {"lpf",                 required_argument,  0, LONG_OPTION_LPF},
            {"lpf-freq",            required_argument,  0, LONG_OPTION_LPF_FRE},
            {"lpf-db",              required_argument,  0, LONG_OPTION_LPF_DB},
            {"eq",                  required_argument,  0, LONG_OPTION_EQ},
            {0,                     0,                  0, 0 }
        };

        c = getopt_long(argc, argv, "D:d:c:r:b:p:v:e:w:G:L:i:o:h",
                 long_options, &option_index);
        if (c == -1)
            break;

        switch (c) {
        case 'D':
            gCardNum = atoi(optarg);
            break;
        case 'd':
            gDeviceNum = atoi(optarg);
            break;
        case 'c':
            gChannels = atoi(optarg);
            break;
        case 'r':
            gRate = atoi(optarg);
            break;
        case 'b':
            gBits = atoi(optarg);
            break;
        case 'p':
            gPeriodSize = atoi(optarg);
            break;
        case 'v':
            gIsWave = atoi(optarg);
            break;
        case 'e':
            gEncoderType = optarg;
            break;
        case 'w':
            gWriteFrames = atoi(optarg);
            break;
        case 'G':
            gGetNumber = atoi(optarg);
            break;
        case 'L':
            gLoopNumber = atoi(optarg);
            break;
        case 'i':
            gInputFile = optarg;
            break;
        case 'o':
            gOutputFile = optarg;
            break;
        case 'h':
            isExit = 1;
            break;
        case LONG_OPTION_AEC_MODE:
            gAecCfg.enAecMode = (AX_AEC_MODE_E)atoi(optarg);
            break;
        case LONG_OPTION_GET_NOISE:
            gGetNoise = atoi(optarg);
            break;
        case LONG_OPTION_SUPPRESSION_LEVEL:
            gAecCfg.stAecFloatCfg.enSuppressionLevel = (AX_SUPPRESSION_LEVEL_E)atoi(optarg);
            break;
        case LONG_OPTION_ROUTING_MODE:
            gAecCfg.stAecFixedCfg.eRoutingMode = (AX_ROUTING_MODE_E)atoi(optarg);
            break;
        case LONG_OPTION_AENC_CHANNELS:
            gAencChannels = atoi(optarg);
            break;
        case LONG_OPTION_LAYOUT_MODE:
            gLayoutMode = (AX_AI_LAYOUT_MODE_E)atoi(optarg);
            break;
        case LONG_OPTION_NS_ENABLE:
            gNsCfg.bNsEnable = (AX_BOOL)atoi(optarg);
            break;
        case LONG_OPTION_AGGRESSIVENESS_LEVEL:
            gNsCfg.enAggressivenessLevel = (AX_AGGRESSIVENESS_LEVEL_E)atoi(optarg);
            break;
        case LONG_OPTION_AGC_ENABLE:
            gAgcCfg.bAgcEnable = (AX_BOOL)atoi(optarg);
            break;
        case LONG_OPTION_TARGET_LEVEL:
            gAgcCfg.s16TargetLevel = atoi(optarg);
            break;
        case LONG_OPTION_GAIN:
            gAgcCfg.s16Gain = atoi(optarg);
            break;
        case LONG_OPTION_RESAMPLE:
            gResample = atoi(optarg);
            break;
        case LONG_OPTION_RESAMPLE_RATE:
            gResRate = atoi(optarg);
            break;
        case LONG_OPTION_VQE_VOLUME:
            gVqeVolume = atof(optarg);
            break;
        case LONG_OPTION_CONVERTER:
            gConverter = atoi(optarg);
            break;
        case LONG_OPTION_AAC_TYPE:
            gAacType = (AX_AAC_TYPE_E)atoi(optarg);
            break;
        case LONG_OPTION_AAC_TRANS_TYPE:
            gTransType = (AX_AAC_TRANS_TYPE_E)atoi(optarg);
            break;
        case LONG_OPTION_ASC_FILE:
            gAscFile = optarg;
            break;
        case LONG_OPTION_LENGTH_FILE:
            gLengthFile = optarg;
            break;
        case LONG_OPTION_SAVE_FILE:
            gSaveFile = atoi(optarg);
            break;
        case LONG_OPTION_CTRL:
            gCtrl = atoi(optarg);
            break;
        case LONG_OPTION_INSTANT:
            gInstant = atoi(optarg);
            break;
        case LONG_OPTION_PERIOD_COUNT:
            gPeriodCount = atoi(optarg);
            break;
        case LONG_OPTION_INSERT_SILENCE:
            gInsertSilence = atoi(optarg);
            break;
        case LONG_OPTION_SIM_DROP:
            gSimDrop = atoi(optarg);
            break;
        case LONG_OPTION_DB_DETECTION:
            gDbDetection = atoi(optarg);
            break;
        case LONG_OPTION_MIX:
            gMix = atoi(optarg);
            break;
        case LONG_OPTION_MIX_FILE:
            gMixFile = optarg;
            break;
        case LONG_OPTION_ASYNC_TEST:
            gAsyncTest = atoi(optarg);
            break;
        case LONG_OPTION_ASYNC_TEST_NAME:
            gAsyncTestName = optarg;
            break;
        case LONG_OPTION_ASYNC_TEST_NUMBER:
            gAsyncTestNumber = atoi(optarg);
        case LONG_OPTION_HPF:
            gHpfCfg.bEnable = (AX_BOOL)atoi(optarg);
            break;
        case LONG_OPTION_HPF_FRE:
            gHpfCfg.s32Freq = atoi(optarg);
            break;
        case LONG_OPTION_HPF_DB:
            gHpfCfg.s32GainDb = atoi(optarg);
            break;
        case LONG_OPTION_LPF:
            gLpfCfg.bEnable = (AX_BOOL)atoi(optarg);
            break;
        case LONG_OPTION_LPF_FRE:
            gLpfCfg.s32Freq = atoi(optarg);
            break;
        case LONG_OPTION_LPF_DB:
            gLpfCfg.s32GainDb = atoi(optarg);
            break;
        case LONG_OPTION_EQ:
            gEqCfg.bEnable = (AX_BOOL)atoi(optarg);
            break;
        default:
            isExit = 1;
            break;
        }
    }
    if (isExit || optind >= argc) {
        PrintHelp();
        exit(0);
    }

    if (!strcmp(argv[optind], "ai")) {
        AudioInput();
    } else {
        printf("Unknown command: %s\n", argv[optind]);
    }

    return 0;
}
