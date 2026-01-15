#include <string.h>
#include <stdio.h>
#include "ax_venc_api.h"
#include "sample_pool.h"
#include "sample_venc_log.h"
#include "sample_cmd_params.h"

#include "ax_venc.h"

/* 参考你原代码里的全局默认值 */
static AX_VENC_MOD_ATTR_T gDefaultModAttr = {
    .enVencType = AX_VENC_MULTI_ENCODER,
    .stModThdAttr.u32TotalThreadNum = 1,
    .stModThdAttr.bExplicitSched = AX_FALSE,
};

AX_S32 SampleAppVencInit(SAMPLE_APP_VENC_CTX_T* pCtx, SAMPLE_VENC_CMD_PARA_T* pCmd)
{
    AX_S32 s32Ret = AX_SUCCESS;

    if (!pCtx || !pCmd)
    {
        SAMPLE_LOG_ERR("SampleAppVencInit: null ptr.\n");
        return -1;
    }

    memset(pCtx, 0, sizeof(*pCtx));
    pCtx->stModAttr = gDefaultModAttr;
    pCtx->stModAttr.stModThdAttr.u32TotalThreadNum = pCmd->encThdNum;

    /* 1) 内存/POOL 初始化：对应你 main 里的 SampleMemInit */
    s32Ret = SampleMemInit(pCmd);
    if (AX_SUCCESS != s32Ret)
    {
        SAMPLE_LOG_ERR("SampleMemInit failed, ret=0x%x\n", s32Ret);
        return s32Ret;
    }
    pCtx->bMemInited = AX_TRUE;

    /* 2) VENC 模块初始化：对应你 main 里的 AX_VENC_Init */
    s32Ret = AX_VENC_Init(&pCtx->stModAttr);
    if (AX_SUCCESS != s32Ret)
    {
        SAMPLE_LOG_ERR("AX_VENC_Init failed, ret=0x%x\n", s32Ret);
        SampleMemDeinit(pCmd);
        pCtx->bMemInited = AX_FALSE;
        return s32Ret;
    }
    pCtx->bVencInited = AX_TRUE;

    return AX_SUCCESS;
}

AX_S32 SampleAppVencDeinit(SAMPLE_APP_VENC_CTX_T* pCtx, SAMPLE_VENC_CMD_PARA_T* pCmd)
{
    AX_S32 s32Ret = AX_SUCCESS;
    AX_S32 s32Ret2 = AX_SUCCESS;

    if (!pCtx || !pCmd)
    {
        SAMPLE_LOG_ERR("SampleAppVencDeinit: null ptr.\n");
        return -1;
    }

    /* 反初始化顺序与 main 一致：先Deinit VENC，再释放内存池 */
    if (pCtx->bVencInited)
    {
        s32Ret = AX_VENC_Deinit();
        if (AX_SUCCESS != s32Ret)
        {
            SAMPLE_LOG_ERR("AX_VENC_Deinit failed, ret=0x%x\n", s32Ret);
        }
        pCtx->bVencInited = AX_FALSE;
    }

    if (pCtx->bMemInited)
    {
        s32Ret2 = SampleMemDeinit(pCmd);
        if (AX_SUCCESS != s32Ret2)
        {
            SAMPLE_LOG_ERR("SampleMemDeinit failed, ret=0x%x\n", s32Ret2);
        }
        pCtx->bMemInited = AX_FALSE;
    }

    return (AX_SUCCESS != s32Ret) ? s32Ret : s32Ret2;
}