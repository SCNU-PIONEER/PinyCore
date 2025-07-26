#pragma once
#include "FreeRTOS.h"
#include "queue.h"

#include <vector>

#include "StmLog.hpp"

class Node {
    inline static uint8_t uidCounter_ = 0;

public:
    uint8_t uid() { return uid_; }

protected:
    Node() : uid_(uidCounter_++) {};
    ~Node() { uidCounter_--; };
    uint8_t uid_;
};

template <typename T> class Topic;

template <typename T> class Publisher : public Node {
public:
    explicit Publisher(Topic<T> *_topic, T *_sourceObject)
            : topic_(_topic), sourceObject_(_sourceObject)
    {
        if (topic_ != nullptr)
            topic_->registerPublisher(this);
        else {
            while (true)
                ;
        }
    }
    ~Publisher()
    {
        if (topic_ != nullptr)
            topic_->cancelPublisher();
    }
    void publish()
    {
        if (topic_ != nullptr)
            topic_->publish();
    }

private:
    friend class Topic<T>;
    Topic<T> *topic_ = nullptr;
    T *sourceObject_ = nullptr;
};

template <typename T> class Subscriber : public Node {
public:
    Subscriber(const UBaseType_t _queueSize, Topic<T> *_topic, T *_targetObject)
            : topic_(_topic)
            , targetObject_(_targetObject)
            , queue_(xQueueCreate(_queueSize, sizeof(T)))
    {
        if (queue_ == nullptr) {
            while (true)
                ; // TODO: queue creation failed
        }
        if (topic_ != nullptr)
            topic_->registerSubscriber(this);
        else {
            while (true)
                ;
        }
    }
    ~Subscriber()
    {
        if (topic_ != nullptr) {
            topic_->cancelSubscriber(this);
            vQueueDelete(queue_);
        }
    }
    QueueHandle_t queue() { return queue_; }

    bool receive()
    {
        return xQueueReceive(queue_, targetObject_, portMAX_DELAY) == pdTRUE;
    }

private:
    friend class Topic<T>;
    Topic<T> *topic_ = nullptr;
    T *targetObject_ = nullptr;
    QueueHandle_t queue_;
};

template <typename T> class Topic {
public:
    explicit Topic(std::string _name) : name_(_name) {}
    std::string name() { return name_; }
    void registerPublisher(Publisher<T> *_publisher)
    {
        publisher_ = _publisher;
        publisher_->topic_ = this;
        LOG::info("Topic", "%s: register publisher uid:%d", _publisher->uid());
    }
    void cancelPublisher()
    {
        publisher_ = nullptr;
        LOG::info("Topic", "%s: cancel publisher", this->name_);
    }
    void registerSubscriber(Subscriber<T> *_subscriber)
    {
        subscribers_.push_back(_subscriber);
        _subscriber->topic_ = this;
        LOG::info("Topic", "%s: register subscriber uid:%d",
                  _subscriber->uid());
    }
    void cancelSubscriber(Subscriber<T> *_subscriber)
    {
        subscribers_.erase(std::remove(subscribers_.begin(), subscribers_.end(),
                                       _subscriber),
                           subscribers_.end());
        LOG::info("Topic", "%s: cancel subscriber uid:%d", _subscriber->uid());
    }
    void publish()
    {
        if (publisher_ == nullptr) {
            LOG::error("Topic", "%s: publisher not registered", this->name_);
            return;
        }
        for (auto &subscriber : subscribers_) {
            if (subscriber != nullptr)
                xQueueSend(subscriber->queue(), publisher_->sourceObject_,
                           portMAX_DELAY);
        }
    }

private:
    std::string name_;
    Publisher<T> *publisher_ = nullptr;
    std::vector<Subscriber<T> *> subscribers_;
};
