#include <cstdio>
#include <cstring>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <opencv2/opencv.hpp>

#include "base/common.hpp"
#include "base/detection.hpp"
#include "base/pose.hpp"
#include "base/stackchan.hpp"
#include "base/http_server.hpp"
#include "base/mqtt_simple.hpp"
#include "base/dumpjson.hpp"
#include "middleware/io.hpp"
#include "utilities/cmdline.hpp"
#include "utilities/file.hpp"
#include "utilities/timer.hpp"

#include <ax_sys_api.h>
#include <ax_venc_api.h>
#include <ax_engine_api.h>

#include "ax_venc.h"
#include "sample_cmd_params.h"
#include <common_venc.h>

using namespace std;

const int DEFAULT_IMG_H = 320;
const int DEFAULT_IMG_W = 320;
const int HAND_IMG_H = 224;
const int HAND_IMG_W = 224;
const int PALM_IN_H = 192;
const int PALM_IN_W = 192;
const int QUEUE_SIZE = 2;

// ====================== 帧队列类 ======================
class FrameQueue
{
    std::queue<cv::Mat> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    size_t max_size_;
    std::atomic<bool> stop_flag_;

public:
    FrameQueue(size_t max_size = QUEUE_SIZE) : max_size_(max_size), stop_flag_(false)
    {
    }

    void push(const cv::Mat& frame)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (queue_.size() >= max_size_ && !stop_flag_) queue_.pop();
        if (!stop_flag_)
        {
            queue_.push(frame.clone());
            cv_.notify_one();
        }
    }

    bool pop(cv::Mat& frame)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || stop_flag_; });
        if (queue_.empty()) return false;
        frame = queue_.front();
        queue_.pop();
        return true;
    }

    void stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_flag_ = true;
        cv_.notify_all();
    }
};

class JsonQueue
{
public:
    enum class OverflowPolicy
    {
        DROP_OLD, // 满了丢最旧的（推荐：保证发送拿到“最新”）
        DROP_NEW  // 满了丢最新的（保证不丢历史）
    };

    JsonQueue(size_t max_size,
              OverflowPolicy policy = OverflowPolicy::DROP_OLD)
        : max_size_(max_size), policy_(policy)
    {
    }

    void push(std::string msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (stop_) return;

        if (q_.size() >= max_size_)
        {
            if (policy_ == OverflowPolicy::DROP_OLD)
            {
                q_.pop_front();
            }
            else
            {
                return; // DROP_NEW
            }
        }
        q_.push_back(std::move(msg));
        cv_.notify_one();
    }

    // 取“最新一条”，并清空前面的（推荐用于 1s 发一次：永远发最新结果）
    bool pop_latest(std::string& out)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (q_.empty()) return false;
        out = std::move(q_.back());
        q_.clear();
        return true;
    }

    // 如果你想 1s 发一批（保留队列里的全部）
    bool pop_all(std::deque<std::string>& out_all)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (q_.empty()) return false;
        out_all.swap(q_);
        return true;
    }

    void stop()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_ = true;
        cv_.notify_all();
    }

    bool stopped() const
    {
        return stop_;
    }

private:
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<std::string> q_;
    size_t max_size_;
    OverflowPolicy policy_;
    std::atomic<bool> stop_{false};
};

JsonQueue* g_json_q = nullptr;

// ====================== Engine Helper（复用 handle） ======================
struct AxEngineContext
{
    AX_ENGINE_HANDLE handle{nullptr};
    AX_ENGINE_IO_INFO_T* io_info{nullptr};
    AX_ENGINE_IO_T io_data{};
    bool inited{false};
};

struct JpegVencCtx
{
    bool bVencInited = false;
    bool bChnCreated = false;
    VENC_CHN chn = 0;
};

static AX_S32 VencJpegOnce_Init(JpegVencCtx& ctx,
                                VENC_CHN VeChn,
                                AX_U32 encThdNum,
                                const AX_VENC_CHN_ATTR_T& chnAttrIn)
{
    ctx = JpegVencCtx{};
    ctx.chn = VeChn;

    // 1) AX_VENC_Init
    AX_VENC_MOD_ATTR_T stModAttr;
    std::memset(&stModAttr, 0, sizeof(stModAttr));
    stModAttr.enVencType = AX_VENC_MULTI_ENCODER;
    stModAttr.stModThdAttr.u32TotalThreadNum = encThdNum;
    stModAttr.stModThdAttr.bExplicitSched = AX_FALSE;

    AX_S32 ret = AX_VENC_Init(&stModAttr);
    if (AX_SUCCESS != ret)
    {
        std::fprintf(stderr, "AX_VENC_Init failed, ret=0x%x\n", ret);
        return ret;
    }
    ctx.bVencInited = true;

    // 2) AX_VENC_CreateChn
    AX_VENC_CHN_ATTR_T chnAttr = chnAttrIn; // 复制一份，避免调用方后续修改
    ret = AX_VENC_CreateChn(VeChn, &chnAttr);
    if (AX_SUCCESS != ret)
    {
        std::fprintf(stderr, "AX_VENC_CreateChn(%d) failed, ret=0x%x\n", VeChn, ret);
        AX_VENC_Deinit();
        ctx = JpegVencCtx{};
        return ret;
    }
    ctx.bChnCreated = true;

    return AX_SUCCESS;
}

static AX_S32 VencJpegOnce_Deinit(JpegVencCtx& ctx)
{
    AX_S32 retAll = AX_SUCCESS;

    if (ctx.bChnCreated)
    {
        AX_S32 ret = AX_VENC_DestroyChn(ctx.chn);
        if (AX_SUCCESS != ret)
        {
            std::fprintf(stderr, "AX_VENC_DestroyChn(%d) failed, ret=0x%x\n", ctx.chn, ret);
            retAll = ret;
        }
        ctx.bChnCreated = false;
    }

    if (ctx.bVencInited)
    {
        AX_S32 ret = AX_VENC_Deinit();
        if (AX_SUCCESS != ret)
        {
            std::fprintf(stderr, "AX_VENC_Deinit failed, ret=0x%x\n", ret);
            retAll = ret;
        }
        ctx.bVencInited = false;
    }

    return retAll;
}

static bool init_engine_once(AxEngineContext& ctx, const std::string& model_path)
{
    if (ctx.inited) return true;

    // 1. init engine (只在第一次使用时调用；AX_SYS_Init 在 main 中统一)
    static bool s_engine_inited = false;
    if (!s_engine_inited)
    {
        AX_ENGINE_NPU_ATTR_T npu_attr;
        memset(&npu_attr, 0, sizeof(npu_attr));
        npu_attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_DISABLE;
        auto ret = AX_ENGINE_Init(&npu_attr);
        if (0 != ret)
        {
            fprintf(stderr, "AX_ENGINE_Init failed, ret = 0x%x\n", ret);
            return false;
        }
        s_engine_inited = true;
    }

    // 2. read model
    std::vector<char> model_buffer;
    if (!utilities::read_file(model_path, model_buffer))
    {
        fprintf(stderr, "Read AX-Engine model(%s) file failed.\n", model_path.c_str());
        return false;
    }

    // 3. create handle
    auto ret = AX_ENGINE_CreateHandle(&ctx.handle,
                                      model_buffer.data(),
                                      model_buffer.size());
    if (0 != ret)
    {
        fprintf(stderr, "AX_ENGINE_CreateHandle failed, ret = 0x%x\n", ret);
        return false;
    }

    // 4. create context
    ret = AX_ENGINE_CreateContext(ctx.handle);
    if (0 != ret)
    {
        fprintf(stderr, "AX_ENGINE_CreateContext failed, ret = 0x%x\n", ret);
        AX_ENGINE_DestroyHandle(ctx.handle);
        ctx.handle = nullptr;
        return false;
    }

    // 5. get io info
    ret = AX_ENGINE_GetIOInfo(ctx.handle, &ctx.io_info);
    if (0 != ret)
    {
        fprintf(stderr, "AX_ENGINE_GetIOInfo failed, ret = 0x%x\n", ret);
        AX_ENGINE_DestroyHandle(ctx.handle);
        ctx.handle = nullptr;
        return false;
    }

    // 6. alloc io
    memset(&ctx.io_data, 0, sizeof(ctx.io_data));
    ret = middleware::prepare_io(ctx.io_info,
                                 &ctx.io_data,
                                 std::make_pair(AX_ENGINE_ABST_DEFAULT,
                                                AX_ENGINE_ABST_CACHED));
    if (0 != ret)
    {
        fprintf(stderr, "prepare_io failed, ret = 0x%x\n", ret);
        AX_ENGINE_DestroyHandle(ctx.handle);
        ctx.handle = nullptr;
        return false;
    }

    ctx.inited = true;
    return true;
}

static inline bool run_engine(AxEngineContext& ctx,
                              const std::vector<uint8_t>& data)
{
    AX_S32 ret = middleware::push_input(data, &ctx.io_data, ctx.io_info);
    if (0 != ret)
    {
        fprintf(stderr, "push_input failed, ret = 0x%x\n", ret);
        return false;
    }
    ret = AX_ENGINE_RunSync(ctx.handle, &ctx.io_data);
    if (0 != ret)
    {
        fprintf(stderr, "AX_ENGINE_RunSync failed, ret = 0x%x\n", ret);
        return false;
    }
    return true;
}

// ====================== Face 检测 ======================
namespace task_face
{
    using namespace detection;

    const char* CLASS_NAMES[] = {"face"};
    int NUM_CLASS = 1;
    const float PROB_THRESHOLD = 0.45f;
    const float NMS_THRESHOLD = 0.45f;

    static AxEngineContext ctx;
    std::string model_file;

    void post_process(const cv::Mat& mat_in,
                      cv::Mat& mat_out,
                      int iw, int ih)
    {
        std::vector<Object> proposals, objects;

        float* output_ptr[3] = {(float*)ctx.io_data.pOutputs[0].pVirAddr,      // 1*80*80*4
                                (float*)ctx.io_data.pOutputs[2].pVirAddr,      // 1*40*40*4
                                (float*)ctx.io_data.pOutputs[4].pVirAddr};     // 1*20*20*4
        float* output_cls_ptr[3] = {(float*)ctx.io_data.pOutputs[1].pVirAddr,  // 1*80*80*80
                                    (float*)ctx.io_data.pOutputs[3].pVirAddr,  // 1*40*40*80
                                    (float*)ctx.io_data.pOutputs[5].pVirAddr}; // 1*20*20*80
        for (int i = 0; i < 3; ++i)
        {
            auto feat_ptr = output_ptr[i];
            auto feat_cls_ptr = output_cls_ptr[i];
            int32_t stride = (1 << i) * 8;
            detection::generate_proposals_yolo26(stride, feat_ptr, feat_cls_ptr, PROB_THRESHOLD, proposals, iw, ih, NUM_CLASS);
        }

        get_out_bbox(proposals, objects, NMS_THRESHOLD,
                     ih, iw, mat_in.rows, mat_in.cols);

        mat_out = draw_objects(mat_in, objects, CLASS_NAMES,
                               "Face Detection", 1.0, 2);
        process_objects_for_servo(objects);
        auto j = objects_envelope_to_json(objects, "Face Detection",
                                          CLASS_NAMES, mat_in.cols, mat_in.rows);
        if (g_json_q)
        {
            g_json_q->push(j.dump());
        }
    }

    bool run(cv::Mat& mat, const std::vector<uint8_t>& data, int h, int w)
    {
        if (!init_engine_once(ctx, model_file)) return false;
        if (!run_engine(ctx, data)) return false;
        post_process(mat, mat, w, h);
        return true;
    }
} // namespace task_face

// ====================== Pose 姿态 ======================
namespace task_pose
{
    using namespace detection;

    const char* CLASS_NAMES[] = {"person"};
    const std::vector<std::vector<uint8_t> > KPS_COLORS = {
        {0, 255, 0}, {0, 255, 0}, {0, 255, 0}, {0, 255, 0}, {0, 255, 0}, {255, 128, 0}, {255, 128, 0}, {255, 128, 0}, {255, 128, 0}, {255, 128, 0}, {255, 128, 0}, {51, 153, 255}, {51, 153, 255}, {51, 153, 255}, {51, 153, 255}, {51, 153, 255}, {51, 153, 255}};
    const std::vector<std::vector<uint8_t> > LIMB_COLORS = {
        {51, 153, 255}, {51, 153, 255}, {51, 153, 255}, {51, 153, 255}, {255, 51, 255}, {255, 51, 255}, {255, 51, 255}, {255, 128, 0}, {255, 128, 0}, {255, 128, 0}, {255, 128, 0}, {255, 128, 0}, {0, 255, 0}, {0, 255, 0}, {0, 255, 0}, {0, 255, 0}, {0, 255, 0}, {0, 255, 0}, {0, 255, 0}};
    const std::vector<std::vector<uint8_t> > SKELETON = {
        {16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12}, {7, 13}, {6, 7}, {6, 8}, {7, 9}, {8, 10}, {9, 11}, {2, 3}, {1, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7}};

    int NUM_CLASS = 1;
    int NUM_POINT = 17;
    const float PROB_THRESHOLD = 0.45f;
    const float NMS_THRESHOLD = 0.45f;

    static AxEngineContext ctx;
    std::string model_file;

    void post_process(const cv::Mat& mat_in,
                      cv::Mat& mat_out,
                      int iw, int ih)
    {
        std::vector<Object> proposals, objects;

        float* out_ptr[3] = {
            (float*)ctx.io_data.pOutputs[0].pVirAddr,
            (float*)ctx.io_data.pOutputs[1].pVirAddr,
            (float*)ctx.io_data.pOutputs[2].pVirAddr};
        float* kps_ptr[3] = {
            (float*)ctx.io_data.pOutputs[3].pVirAddr,
            (float*)ctx.io_data.pOutputs[4].pVirAddr,
            (float*)ctx.io_data.pOutputs[5].pVirAddr};

        for (int i = 0; i < 3; ++i)
        {
            int stride = (1 << i) * 8;
            generate_proposals_yolov8_pose_native(stride,
                                                  out_ptr[i],
                                                  kps_ptr[i],
                                                  PROB_THRESHOLD,
                                                  proposals,
                                                  iw,
                                                  ih,
                                                  NUM_POINT,
                                                  NUM_CLASS);
        }

        get_out_bbox_kps(proposals, objects, NMS_THRESHOLD,
                         ih, iw, mat_in.rows, mat_in.cols);

        mat_out = draw_keypoints(mat_in, objects,
                                 KPS_COLORS, LIMB_COLORS, SKELETON,
                                 "Pose", 1.0, 2);
    }

    bool run(cv::Mat& mat, const std::vector<uint8_t>& data, int h, int w)
    {
        if (!init_engine_once(ctx, model_file)) return false;
        if (!run_engine(ctx, data)) return false;
        post_process(mat, mat, w, h);
        return true;
    }
} // namespace task_pose

// ====================== Palm + HandPose ======================
namespace task_hand
{
    using namespace detection;

    const int HAND_JOINTS = 21;
    const float PROB_THRESHOLD = 0.65f;
    const float NMS_THRESHOLD = 0.45f;
    const int map_size[2] = {24, 12};
    const int strides[2] = {8, 16};
    const int anchor_size[2] = {2, 6};
    const float anchor_offset[2] = {0.5f, 0.5f};

    static AxEngineContext palm_ctx;
    static AxEngineContext hand_ctx;
    std::string palm_model_file, hand_model_file;

    bool init_palm()
    {
        return init_engine_once(palm_ctx, palm_model_file);
    }
    bool init_hand()
    {
        return init_engine_once(hand_ctx, hand_model_file);
    }

    bool run_hand_model(const std::vector<uint8_t>& data,
                        pose::ai_hand_parts_s& out_pose)
    {
        if (!init_hand()) return false;

        AX_S32 ret = middleware::push_input(data, &hand_ctx.io_data, hand_ctx.io_info);
        if (0 != ret)
        {
            fprintf(stderr, "Hand push_input failed, ret = 0x%x\n", ret);
            return false;
        }
        ret = AX_ENGINE_RunSync(hand_ctx.handle, &hand_ctx.io_data);
        if (0 != ret)
        {
            fprintf(stderr, "Hand AX_ENGINE_RunSync failed, ret = 0x%x\n", ret);
            return false;
        }

        auto& info_point = hand_ctx.io_data.pOutputs[0];
        auto& info_score = hand_ctx.io_data.pOutputs[1];
        float* point_ptr = (float*)info_point.pVirAddr;
        float* score_ptr = (float*)info_score.pVirAddr;
        pose::post_process_hand(point_ptr, score_ptr,
                                out_pose, HAND_JOINTS,
                                HAND_IMG_H, HAND_IMG_W);
        return true;
    }

    void post_process_palm(cv::Mat& mat)
    {
        std::vector<PalmObject> proposals, objects;
        auto bboxes_ptr = (float*)palm_ctx.io_data.pOutputs[0].pVirAddr;
        auto scores_ptr = (float*)palm_ctx.io_data.pOutputs[1].pVirAddr;
        float prob_threshold_unsigmoid = -1.0f * (float)std::log((1.0f / PROB_THRESHOLD) - 1.0f);

        generate_proposals_palm(proposals,
                                PROB_THRESHOLD,
                                PALM_IN_W,
                                PALM_IN_H,
                                scores_ptr,
                                bboxes_ptr,
                                2,
                                strides,
                                anchor_size,
                                anchor_offset,
                                map_size,
                                prob_threshold_unsigmoid);

        get_out_bbox_palm(proposals, objects, NMS_THRESHOLD,
                          PALM_IN_H, PALM_IN_W, mat.rows, mat.cols);

        cv::Mat mat_draw = mat;

        static int last_gesture_id = -1;
        static int same_count = 0;
        const int required_count = 2;

        using Clock = std::chrono::steady_clock;
        static std::unordered_map<std::string, Clock::time_point> last_fire_time;

        auto cooldown_for = [](const std::string& label) -> std::chrono::milliseconds {
            if (label == "ok" || label == "five") return std::chrono::milliseconds(3000);
            return std::chrono::milliseconds(1000);
        };

        auto can_fire = [&](const std::string& label) -> bool {
            const auto now = Clock::now();
            const auto cd = cooldown_for(label);
            auto it = last_fire_time.find(label);
            if (it == last_fire_time.end() || (now - it->second) >= cd)
            {
                last_fire_time[label] = now;
                return true;
            }
            return false;
        };

        bool fired_this_frame = false;

        for (size_t i = 0; i < objects.size(); ++i)
        {
            if (fired_this_frame) break;

            cv::Mat hand_roi;
            cv::warpAffine(mat, hand_roi,
                           objects[i].affine_trans_mat,
                           cv::Size(HAND_IMG_W, HAND_IMG_H));

            std::vector<uint8_t> hand_image(HAND_IMG_H * HAND_IMG_W * 3);
            common::get_input_data_no_letterbox(hand_roi,
                                                hand_image,
                                                HAND_IMG_H,
                                                HAND_IMG_W,
                                                true);

            pose::ai_hand_parts_s hand_parts;
            if (!run_hand_model(hand_image, hand_parts))
                continue;

            pose::draw_result_hand_on_image(mat_draw,
                                            hand_parts,
                                            HAND_JOINTS,
                                            objects[i].affine_trans_mat_inv);

            if (check_palm_objects_size(objects, 0.05, 0.05) != 0)
                continue;

            int gesture_id = classify_gesture(hand_parts);
            if (gesture_id == -1)
                continue;

            if (gesture_id == last_gesture_id)
                same_count++;
            else
            {
                same_count = 1;
                last_gesture_id = gesture_id;
            }

            if (same_count < required_count)
                continue;

            same_count = 0;

            const std::string& label = gesture_defs[gesture_id].label;

            if (!can_fire(label))
                continue;

            if (label == "ok")
            {
                // send_motion("reverse");
                send_motion("forward");
            }
            else if (label == "one")
            {
                send_motion("shake");
            }
            else if (label == "two")
            {
                send_motion("nod");
            }
            else if (label == "five")
            {
                send_motion("forward");
            }

            last_gesture_id = -1;
            same_count = 0;

            fired_this_frame = true;
            break;
        }

        mat = draw_objects_palm(mat_draw, objects, "Palm detection");
    }

    bool run(cv::Mat& mat, const std::vector<uint8_t>& data)
    {
        if (!init_palm()) return false;

        AX_S32 ret = middleware::push_input(data, &palm_ctx.io_data, palm_ctx.io_info);
        if (0 != ret)
        {
            fprintf(stderr, "Palm push_input failed, ret = 0x%x\n", ret);
            return false;
        }
        ret = AX_ENGINE_RunSync(palm_ctx.handle, &palm_ctx.io_data);
        if (0 != ret)
        {
            fprintf(stderr, "Palm AX_ENGINE_RunSync failed, ret = 0x%x\n", ret);
            return false;
        }

        post_process_palm(mat);
        return true;
    }
} // namespace task_hand

// ====================== 采集线程 ======================
void captureFrames(cv::VideoCapture& cap, FrameQueue& fq, std::atomic<bool>& stop)
{
    cv::Mat frame;
    while (!stop)
    {
        static uint64_t frame_id = 0;
        const uint64_t skip = 3;

        frame_id++;
        if (frame_id % skip != 0)
        {
            continue;
        }
        cap >> frame;
        if (frame.empty())
        {
            stop = true;
            break;
        }
        cv::flip(frame, frame, 1);
        fq.push(frame);
    }
    fq.stop();
}

void send_jpeg(void* buff, int size);

static void BgrToNv12(const cv::Mat& bgr, std::vector<uint8_t>& nv12)
{
    CV_Assert(!bgr.empty());
    CV_Assert(bgr.type() == CV_8UC3);
    CV_Assert((bgr.cols % 2) == 0 && (bgr.rows % 2) == 0);

    const int w = bgr.cols;
    const int h = bgr.rows;

    cv::Mat i420(h * 3 / 2, w, CV_8UC1);
    cv::cvtColor(bgr, i420, cv::COLOR_BGR2YUV_I420);

    nv12.resize(w * h * 3 / 2);

    uint8_t* dstY = nv12.data();
    uint8_t* dstUV = nv12.data() + w * h;

    const uint8_t* srcY = i420.ptr<uint8_t>(0);
    const uint8_t* srcU = srcY + w * h;
    const uint8_t* srcV = srcU + (w * h) / 4;

    std::memcpy(dstY, srcY, w * h);

    for (int j = 0; j < h / 2; ++j)
    {
        for (int i = 0; i < w / 2; ++i)
        {
            dstUV[j * w + 2 * i + 0] = srcU[j * (w / 2) + i];
            dstUV[j * w + 2 * i + 1] = srcV[j * (w / 2) + i];
        }
    }
}

static int EncodeBgrToJpegAndSend(const cv::Mat& bgr, int qfactor)
{
    if (bgr.empty() || bgr.type() != CV_8UC3) return -1;
    if (bgr.cols != 1280 || bgr.rows != 720)
    {
        return -2;
    }
    if ((bgr.cols % 2) || (bgr.rows % 2)) return -3;

    const uint32_t w = (uint32_t)bgr.cols;
    const uint32_t h = (uint32_t)bgr.rows;

    std::vector<uint8_t> nv12;
    BgrToNv12(bgr, nv12);

    const uint32_t strideY = w;
    const uint32_t strideUV = w;
    const uint32_t inSize = strideY * h + strideUV * (h / 2);

    AX_U64 inPhy = 0;
    AX_VOID* inVir = nullptr;
    int ret = AX_SYS_MemAlloc(&inPhy, &inVir, inSize, 0, (AX_S8*)"JENC_IN_NV12");
    if (ret != AX_SUCCESS) return -10;

    std::memcpy(inVir, nv12.data(), inSize);

    const uint32_t strmBufSize = inSize;

    AX_U64 outPhy = 0;
    AX_VOID* outVir = nullptr;
    ret = AX_SYS_MemAllocCached(&outPhy, &outVir, strmBufSize, 0, (AX_S8*)"JENC_OUT");
    if (ret != AX_SUCCESS)
    {
        AX_SYS_MemFree(inPhy, inVir);
        return -11;
    }

    AX_JPEG_ENCODE_ONCE_PARAMS_T p;
    std::memset(&p, 0, sizeof(p));

    p.u32Width = w;
    p.u32Height = h;
    p.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    p.stJpegParam.u32Qfactor = 55;
    p.stJpegParam.bDblkEnable = AX_FALSE;

    p.u64PhyAddr[0] = inPhy;
    p.u64PhyAddr[1] = inPhy + strideY * h;
    p.u64PhyAddr[2] = 0;

    p.u32PicStride[0] = strideY;
    p.u32PicStride[1] = strideUV;
    p.u32PicStride[2] = 0;

    p.enStrmBufType = AX_STREAM_BUF_CACHE;
    p.ulPhyAddr = outPhy;
    p.pu8Addr = (AX_U8*)outVir;
    p.u32Len = strmBufSize;

    ret = AX_VENC_JpegEncodeOneFrame(&p);
    if (ret != AX_SUCCESS)
    {
        AX_SYS_MemFree(outPhy, outVir);
        AX_SYS_MemFree(inPhy, inVir);
        return -12;
    }

    std::vector<uint8_t> jpg;
    jpg.resize(p.u32Len);
    std::memcpy(jpg.data(), p.pu8Addr, p.u32Len);

    send_jpeg(jpg.data(), jpg.size());

    AX_SYS_MemFree(outPhy, outVir);
    AX_SYS_MemFree(inPhy, inVir);
    return 0;
}

static void json_sender_thread(MqttSimplePublisher& mqtt_pub,
                               JsonQueue& jq,
                               std::atomic<bool>& stop,
                               std::chrono::milliseconds interval)
{
    using namespace std::chrono;

    constexpr auto kConnectWaitTotal = 1500ms; // 总共等待连接完成的时间
    constexpr auto kConnectWaitStep = 50ms;    // 每次轮询间隔
    constexpr auto kAfterConnectDelay = 100ms; // 连接后额外再等一下（给握手/loop一点时间）

    while (!stop)
    {
        std::this_thread::sleep_for(interval);

        std::string payload;
        if (!jq.pop_latest(payload)) continue;

        std::printf("JSON SEND (%zu bytes): %s\n", payload.size(), payload.c_str());

        if (!mqtt_pub.is_connected())
        {
            std::fprintf(stderr, "[MQTT] not connected, try connect...\n");
            mqtt_pub.connect("yolo_face_sender");

            // 等待连接状态变为 connected（避免 connect 异步导致立刻 publish 失败）
            auto waited = 0ms;
            while (!stop && !mqtt_pub.is_connected() && waited < kConnectWaitTotal)
            {
                std::this_thread::sleep_for(kConnectWaitStep);
                waited += kConnectWaitStep;
            }

            if (mqtt_pub.is_connected())
            {
                std::fprintf(stderr, "[MQTT] connected (waited %lld ms)\n",
                             (long long)waited.count());
                std::this_thread::sleep_for(kAfterConnectDelay);
            }
            else
            {
                std::fprintf(stderr, "[MQTT] connect timeout after %lld ms\n",
                             (long long)waited.count());
                continue; // 没连上就跳过这次发送，避免必然失败
            }
        }

        if (!mqtt_pub.publish_json(payload))
        {
            std::fprintf(stderr, "MQTT publish failed (connected=%d)\n",
                         mqtt_pub.is_connected() ? 1 : 0);
        }
    }
}
// ====================== 主程序 ======================
int main(int argc, char** argv)
{
    cmdline::parser cmd;
    cmd.add<std::string>("face_model", 'f', "face model(joint)", true, "");
    cmd.add<std::string>("pose_model", 'p', "pose model(joint)", true, "");
    cmd.add<std::string>("palm_model", 'm', "palm model(joint)", true, "");
    cmd.add<std::string>("hand_model", 'h', "hand model(joint)", true, "");
    cmd.add<std::string>("video", 'v', "video src", true, "");
    cmd.add<int>("json_queue_size", 0, "json queue size", false, 30);
    cmd.add<int>("json_interval_ms", 0, "json send interval(ms)", false, 1000);
    cmd.add<std::string>("mqtt_host", 0, "mqtt host", false, "192.168.3.3");
    cmd.add<int>("mqtt_port", 0, "mqtt port", false, 1883);
    cmd.add<std::string>("mqtt_topic", 0, "mqtt topic", false, "yolo/face/data");
    cmd.parse_check(argc, argv);

    task_face::model_file = cmd.get<std::string>("face_model");
    task_pose::model_file = cmd.get<std::string>("pose_model");
    task_hand::palm_model_file = cmd.get<std::string>("palm_model");
    task_hand::hand_model_file = cmd.get<std::string>("hand_model");
    std::string video_src = cmd.get<std::string>("video");

    // 打开摄像头 / 视频
    cv::VideoCapture cap;
    try
    {
        int idx = std::stoi(video_src);
        cap.open(idx, cv::CAP_V4L2);
    }
    catch (...)
    {
        cap.open(video_src);
    }
    if (!cap.isOpened())
    {
        fprintf(stderr, "Video open failed.\n");
        return -1;
    }

    int mjpg = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    cap.set(cv::CAP_PROP_FOURCC, mjpg);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 30);

    int json_q_size = cmd.get<int>("json_queue_size");
    int json_interval_ms = cmd.get<int>("json_interval_ms");

    JsonQueue jq((size_t)json_q_size, JsonQueue::OverflowPolicy::DROP_OLD);
    g_json_q = &jq;

    const std::string mqtt_host = cmd.get<std::string>("mqtt_host");
    const int mqtt_port = cmd.get<int>("mqtt_port");
    const std::string mqtt_topic = cmd.get<std::string>("mqtt_topic");

    MqttSimplePublisher mqtt_pub(
        mqtt_host,
        static_cast<std::uint16_t>(mqtt_port),
        mqtt_topic);

    std::atomic<bool> stop_sender(false);

    std::thread t_sender(json_sender_thread,
                         std::ref(mqtt_pub),
                         std::ref(jq),
                         std::ref(stop_sender),
                         std::chrono::milliseconds(json_interval_ms));

    AX_S32 ret
        = AX_SYS_Init();
    if (0 != ret)
    {
        fprintf(stderr, "AX_SYS_Init failed, ret = 0x%x\n", ret);
        return ret;
    }

    SAMPLE_VENC_CMD_PARA_T venccmd;
    memset(&venccmd, 0, sizeof(venccmd));
    SampleSetDefaultParams(&venccmd);

    venccmd.encThdNum = 1;
    venccmd.logLevel = AX_TRUE;

    venccmd.picW = 1280;
    venccmd.picH = 720;
    venccmd.picFormat = AX_FORMAT_YUV420_SEMIPLANAR;

    auto ALIGN_UP = [](uint32_t x, uint32_t a) { return (x + a - 1) / a * a; };
    venccmd.strideY = ALIGN_UP(venccmd.picW, 64);
    venccmd.strideU = venccmd.strideY;
    venccmd.strideV = 0;

    venccmd.frameSize = venccmd.strideY * venccmd.picH + venccmd.strideU * (venccmd.picH / 2);

    venccmd.vbCnt = 4;

    venccmd.poolId = AX_INVALID_POOLID;

    venccmd.qpMapQpType = 0;
    venccmd.qpMapBlkType = 0;
    venccmd.qpMapSize = 0;

    SAMPLE_APP_VENC_CTX_T ctx;
    ret = SampleAppVencInit(&ctx, &venccmd);
    if (AX_SUCCESS != ret)
    {
        fprintf(stderr, "SampleAppVencInit failed, ret=0x%x\n", ret);
        return -1;
    }

    FrameQueue fq(QUEUE_SIZE);
    std::atomic<bool> stop(false);
    std::thread t_cap(captureFrames, std::ref(cap), std::ref(fq), std::ref(stop));

    cv::Mat frame;
    std::vector<uint8_t> resized(DEFAULT_IMG_H * DEFAULT_IMG_W * 3);
    std::vector<uint8_t> hand_resized(PALM_IN_H * PALM_IN_W * 3);

    serial_init("/dev/ttyACM0");

    struct PerfInfo
    {
        std::string name;
        double time_ms;
    };

    std::vector<PerfInfo> perf_stats{
        {"YOLO11n-Detect", 0},
        {"YOLO11n-Pose", 0},
        {"HandPose", 0},
    };

    auto update_time_only = [](PerfInfo& p, double ms) { p.time_ms = ms; };

    auto draw_perf = [](cv::Mat& mat, const std::vector<PerfInfo>& stats) {
        int x = 10, y = 25;
        cv::Mat overlay;
        mat.copyTo(overlay);
        cv::rectangle(overlay,
                      cv::Point(0, 0),
                      cv::Point(220, stats.size() * 25 + 15),
                      cv::Scalar(0, 0, 0),
                      -1);
        double alpha = 0.4;
        cv::addWeighted(overlay, alpha, mat, 1 - alpha, 0, mat);
        const double font_scale = 0.5;
        const int thickness = 1;
        const int outline_thickness = 2;
        for (auto& p : stats)
        {
            char buf[100];
            snprintf(buf, sizeof(buf), "%s: %.1f ms", p.name.c_str(), p.time_ms);
            cv::putText(mat, buf, cv::Point(x + 1, y + 1),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        cv::Scalar(0, 0, 0), outline_thickness);
            cv::putText(mat, buf, cv::Point(x, y),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale,
                        cv::Scalar(0, 255, 255), thickness);
            y += 25;
        }
    };

    // 预热：确保三个模型的 Engine 已经创建
    {
        std::vector<uint8_t> dummy_face(DEFAULT_IMG_H * DEFAULT_IMG_W * 3, 0);
        std::vector<uint8_t> dummy_palm(PALM_IN_H * PALM_IN_W * 3, 0);
        cv::Mat dummy(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        task_face::run(dummy, dummy_face, DEFAULT_IMG_H, DEFAULT_IMG_W);
        task_pose::run(dummy, dummy_face, DEFAULT_IMG_H, DEFAULT_IMG_W);
        task_hand::run(dummy, dummy_palm);
    }

    while (!stop)
    {
        if (!fq.pop(frame)) break;

        cv::Mat canvas = frame.clone();

        common::get_input_data_letterbox(frame, resized,
                                         DEFAULT_IMG_H, DEFAULT_IMG_W,
                                         true);
        common::get_input_data_letterbox(frame, hand_resized,
                                         PALM_IN_H, PALM_IN_W,
                                         true);

        // Face
        {
            int64 start = cv::getTickCount();
            task_face::run(canvas, resized, DEFAULT_IMG_H, DEFAULT_IMG_W);
            double elapsed_ms = (cv::getTickCount() - start) * 1000.0 / cv::getTickFrequency();
            update_time_only(perf_stats[0], elapsed_ms);
        }

        // Pose
        {
            int64 start = cv::getTickCount();
            task_pose::run(canvas, resized, DEFAULT_IMG_H, DEFAULT_IMG_W);
            double elapsed_ms = (cv::getTickCount() - start) * 1000.0 / cv::getTickFrequency();
            update_time_only(perf_stats[1], elapsed_ms);
        }

        // Hand (Palm + HandPose)
        {
            int64 start = cv::getTickCount();
            task_hand::run(canvas, hand_resized);
            double elapsed_ms = (cv::getTickCount() - start) * 1000.0 / cv::getTickFrequency();
            update_time_only(perf_stats[2], elapsed_ms);
        }

        draw_perf(canvas, perf_stats);

        int ret = 0;
        ret = EncodeBgrToJpegAndSend(canvas, 45);
        if (ret)
        {
            fprintf(stderr, "EncodeBgrToJpegAndSend failed.%d\n", ret);
        }

        // {
        //     std::vector<int> params;
        //     params.push_back(cv::IMWRITE_JPEG_QUALITY);
        //     params.push_back(85);

        //     static uint64_t frame_id = 0;
        //     const uint64_t skip = 3;

        //     frame_id++;
        //     if (frame_id % skip != 0)
        //     {
        //         continue;
        //     }

        //     std::vector<uchar> buf;
        //     if (cv::imencode(".jpg", canvas, buf, params))
        //     {
        //         send_jpeg(buf.data(), buf.size());
        //     }
        // }

        // {
        //     cv::imshow("YOLO11 AX-Engine Demo", canvas);
        //     char key = (char)cv::waitKey(1);
        //     if (key == 27 || key == 'q')
        //     {
        //         stop = true;
        //         break;
        //     }
        // }
    }

    stop_sender = true;
    jq.stop();
    if (t_sender.joinable()) t_sender.join();

    fq.stop();
    if (t_cap.joinable()) t_cap.join();
    cap.release();
    cv::destroyAllWindows();

    auto free_ctx = [](AxEngineContext& c) {
        if (!c.inited) return;
        middleware::free_io(&c.io_data);
        if (c.handle) AX_ENGINE_DestroyHandle(c.handle);
        c.handle = nullptr;
        c.io_info = nullptr;
        c.inited = false;
    };
    free_ctx(task_face::ctx);
    free_ctx(task_pose::ctx);
    free_ctx(task_hand::palm_ctx);
    free_ctx(task_hand::hand_ctx);

    SampleAppVencDeinit(&ctx, &venccmd);

    AX_ENGINE_Deinit();
    AX_SYS_Deinit();
    return 0;
}