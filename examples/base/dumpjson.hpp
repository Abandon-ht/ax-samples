#include "base/detection.hpp"
#include <base/json.hpp>

using json = nlohmann::json;

// 新增：detection::Object 的序列化
static inline json object_to_json(const detection::Object& o,
                                  const char* const* class_names = nullptr)
{
    json j;
    j["label"] = o.label;
    if (class_names) j["label_name"] = class_names[o.label];
    j["prob"] = o.prob;
    // j["rect"] = {{"x", o.rect.x}, {"y", o.rect.y}, {"w", o.rect.width}, {"h", o.rect.height}};
    j["rect"] = {o.rect.x, o.rect.y, o.rect.width, o.rect.height};
    return j;
}

// 改成接收 detection::Object
static inline json objects_envelope_to_json(const std::vector<detection::Object>& objects,
                                            const std::string& algo_name,
                                            const char* const* class_names,
                                            int frame_w, int frame_h)
{
    json out;
    out["type"] = algo_name;
    // out["frame"] = {{"w", frame_w}, {"h", frame_h}};
    // out["count"] = (int)objects.size();

    auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now());
    out["ts_ms"] = now.time_since_epoch().count();

    json arr = json::array();
    for (auto& o : objects)
        arr.push_back(object_to_json(o, class_names)); // 现在能匹配到 detection::Object 版本
    out["objects"] = arr;
    return out;
}