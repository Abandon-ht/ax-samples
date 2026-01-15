#pragma once
#include "common_venc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    AX_BOOL bMemInited;
    AX_BOOL bVencInited;
    AX_VENC_MOD_ATTR_T stModAttr;
} SAMPLE_APP_VENC_CTX_T;

AX_S32 SampleAppVencInit(SAMPLE_APP_VENC_CTX_T* pCtx, SAMPLE_VENC_CMD_PARA_T* pCmd);
AX_S32 SampleAppVencDeinit(SAMPLE_APP_VENC_CTX_T* pCtx, SAMPLE_VENC_CMD_PARA_T* pCmd);

#ifdef __cplusplus
}
#endif