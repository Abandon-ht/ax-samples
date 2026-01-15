#pragma once
#include <string>
#include <cstdint>
#include <memory>
#include <iostream>
#include <utility>
#include <type_traits>
#include <atomic>
#include <thread>
#include <chrono>

#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <base/mqtt_client_cpp.hpp>

class MqttSimplePublisher
{
public:
    explicit MqttSimplePublisher(std::string host,
                                 std::uint16_t port = 1883,
                                 std::string topic = "yolo/face/data")
        : host_(std::move(host)), port_(port), topic_(std::move(topic)), work_guard_(boost::asio::make_work_guard(ioc_))
    {
        using namespace MQTT_NS;
        setup_log();

        c_ = make_sync_client(ioc_, host_, std::to_string(port_));

        c_->set_connack_handler([this](bool sp, connect_return_code rc) {
            std::cerr << "[MQTT] connack sp=" << std::boolalpha << sp
                      << " rc=" << connect_return_code_to_str(rc) << "\n";
            connected_.store(rc == connect_return_code::accepted, std::memory_order_release);
            return true;
        });

        c_->set_close_handler([this] {
            std::cerr << "[MQTT] closed\n";
            connected_.store(false, std::memory_order_release);
        });

        c_->set_error_handler([this](MQTT_NS::error_code ec) {
            std::cerr << "[MQTT] error: " << ec.message() << "\n";
            connected_.store(false, std::memory_order_release);
        });

        // 关键：启动 ioc.run()
        ioc_thread_ = std::thread([this] {
            ioc_.run();
        });
    }

    bool connect(std::string client_id = "cid_yolo_face_pub",
                 bool clean_session = true,
                 std::chrono::milliseconds wait_timeout = std::chrono::milliseconds(1500))
    {
        if (!c_) return false;

        connected_.store(false, std::memory_order_release);

        c_->set_client_id(client_id);
        c_->set_clean_session(clean_session);

        std::cerr << "[MQTT] connect() start host=" << host_
                  << " port=" << port_ << " client_id=" << client_id
                  << " clean=" << std::boolalpha << clean_session << "\n";

        try
        {
            c_->connect(); // 异步推进依赖 ioc_ 线程，connack_handler 会置 connected_
        }
        catch (std::exception const& e)
        {
            std::cerr << "[MQTT] connect exception: " << e.what() << "\n";
            return false;
        }

        // 等待 connack（最多 wait_timeout）
        auto waited = std::chrono::milliseconds(0);
        while (waited < wait_timeout)
        {
            if (is_connected())
            {
                std::cerr << "[MQTT] connected (waited " << waited.count() << " ms)\n";
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            waited += std::chrono::milliseconds(50);
        }

        std::cerr << "[MQTT] connect timeout after " << waited.count() << " ms\n";
        return false;
    }

    bool is_connected() const
    {
        return connected_.load(std::memory_order_acquire) && c_ && c_->connected();
    }

    bool publish_json(const std::string& payload)
    {
        if (!is_connected()) return false;

        try
        {
            c_->publish(topic_, payload, MQTT_NS::qos::at_most_once);
            return true;
        }
        catch (std::exception const& e)
        {
            std::cerr << "[MQTT] publish exception: " << e.what() << "\n";
            connected_.store(false, std::memory_order_release);
            return false;
        }
    }

    void disconnect()
    {
        if (c_ && c_->connected())
        {
            try
            {
                c_->disconnect();
            }
            catch (...)
            {
            }
        }
        connected_.store(false, std::memory_order_release);
    }

    ~MqttSimplePublisher()
    {
        disconnect();

        work_guard_.reset();
        ioc_.stop();

        if (ioc_thread_.joinable()) ioc_thread_.join();
    }

private:
    std::string host_;
    std::uint16_t port_;
    std::string topic_;

    std::atomic<bool> connected_{false};

    boost::asio::io_context ioc_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread ioc_thread_;

    using client_ptr_t = decltype(MQTT_NS::make_sync_client(
        std::declval<boost::asio::io_context&>(),
        std::declval<std::string const&>(),
        std::declval<std::string const&>()));
    client_ptr_t c_;
};