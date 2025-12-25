/**
 * @file template_publisher.cpp
 * @brief Demonstration of C++ templates with generic publisher pattern
 *
 * This example demonstrates:
 * - Function templates
 * - Class templates
 * - Template specialization
 * - Generic publisher/subscriber pattern
 * - STL template usage
 *
 * Compile: g++ -std=c++17 -Wall template_publisher.cpp -o template_publisher
 * Run: ./template_publisher
 */

#include <iostream>
#include <vector>
#include <map>
#include <functional>
#include <memory>
#include <string>
#include <typeinfo>
#include <sstream>

// ============================================================================
// Function Templates
// ============================================================================

// Basic function template
template<typename T>
T get_max(T a, T b) {
    return (a > b) ? a : b;
}

// Template with multiple type parameters
template<typename T, typename U>
auto multiply(T a, U b) -> decltype(a * b) {
    return a * b;
}

// Template with non-type parameter
template<typename T, size_t N>
class FixedArray {
public:
    T& operator[](size_t index) { return data_[index]; }
    constexpr size_t size() const { return N; }

private:
    T data_[N];
};

void demo_function_templates() {
    std::cout << "\n=== Function Templates ===\n";

    // Template automatically deduced
    std::cout << "max(10, 20) = " << get_max(10, 20) << "\n";
    std::cout << "max(3.14, 2.71) = " << get_max(3.14, 2.71) << "\n";
    std::cout << "max('a', 'z') = " << get_max('a', 'z') << "\n";

    // Multiple type parameters
    std::cout << "multiply(3, 4.5) = " << multiply(3, 4.5) << "\n";

    // Fixed-size array
    FixedArray<int, 5> arr;
    arr[0] = 10;
    arr[1] = 20;
    std::cout << "Fixed array size: " << arr.size() << "\n";
}

// ============================================================================
// Generic Message Types
// ============================================================================

/**
 * @brief String message type
 */
struct StringMsg {
    std::string data;

    std::string to_string() const {
        return "StringMsg: " + data;
    }
};

/**
 * @brief Integer message type
 */
struct IntMsg {
    int value;

    std::string to_string() const {
        return "IntMsg: " + std::to_string(value);
    }
};

/**
 * @brief Sensor data message type
 */
struct SensorMsg {
    std::string sensor_id;
    double value;
    bool valid;

    std::string to_string() const {
        std::ostringstream oss;
        oss << "SensorMsg[" << sensor_id << "]: " << value
            << (valid ? " (valid)" : " (invalid)");
        return oss.str();
    }
};

// ============================================================================
// Generic Publisher Template
// ============================================================================

/**
 * @brief Generic publisher that works with any message type
 */
template<typename MessageT>
class Publisher {
public:
    using Callback = std::function<void(const MessageT&)>;

    Publisher(const std::string& topic) : topic_(topic) {
        std::cout << "Publisher<" << typeid(MessageT).name()
                  << "> created for topic: " << topic_ << "\n";
    }

    void publish(const MessageT& msg) {
        std::cout << "[" << topic_ << "] Publishing: " << msg.to_string() << "\n";
        for (const auto& callback : subscribers_) {
            callback(msg);
        }
    }

    void subscribe(Callback callback) {
        subscribers_.push_back(callback);
    }

    std::string topic() const { return topic_; }

private:
    std::string topic_;
    std::vector<Callback> subscribers_;
};

void demo_generic_publisher() {
    std::cout << "\n=== Generic Publisher ===\n";

    // Publisher for string messages
    Publisher<StringMsg> string_pub("string_topic");
    string_pub.subscribe([](const StringMsg& msg) {
        std::cout << "  Subscriber received: " << msg.data << "\n";
    });
    string_pub.publish({"Hello, Templates!"});

    // Publisher for integer messages
    Publisher<IntMsg> int_pub("int_topic");
    int_pub.subscribe([](const IntMsg& msg) {
        std::cout << "  Subscriber received: " << msg.value << "\n";
    });
    int_pub.publish({42});

    // Publisher for sensor messages
    Publisher<SensorMsg> sensor_pub("sensor_topic");
    sensor_pub.subscribe([](const SensorMsg& msg) {
        std::cout << "  Subscriber got sensor data\n";
    });
    sensor_pub.publish({"GPS", 37.7749, true});
}

// ============================================================================
// Template Class with Data Container
// ============================================================================

/**
 * @brief Generic circular buffer
 */
template<typename T, size_t Capacity = 10>
class CircularBuffer {
public:
    void push(const T& item) {
        buffer_[write_pos_] = item;
        write_pos_ = (write_pos_ + 1) % Capacity;
        if (size_ < Capacity) {
            size_++;
        }
    }

    T get(size_t index) const {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        size_t pos = (write_pos_ + Capacity - size_ + index) % Capacity;
        return buffer_[pos];
    }

    size_t size() const { return size_; }
    constexpr size_t capacity() const { return Capacity; }

    void print() const {
        std::cout << "Buffer[" << size_ << "/" << Capacity << "]: ";
        for (size_t i = 0; i < size_; ++i) {
            std::cout << get(i) << " ";
        }
        std::cout << "\n";
    }

private:
    T buffer_[Capacity];
    size_t write_pos_ = 0;
    size_t size_ = 0;
};

void demo_circular_buffer() {
    std::cout << "\n=== Circular Buffer Template ===\n";

    CircularBuffer<int, 5> int_buffer;
    for (int i = 1; i <= 7; ++i) {
        int_buffer.push(i);
        int_buffer.print();
    }

    CircularBuffer<std::string, 3> string_buffer;
    string_buffer.push("first");
    string_buffer.push("second");
    string_buffer.push("third");
    string_buffer.push("fourth");  // Overwrites "first"
    string_buffer.print();
}

// ============================================================================
// Template Specialization
// ============================================================================

/**
 * @brief Generic statistics calculator
 */
template<typename T>
class Statistics {
public:
    void add(const T& value) {
        values_.push_back(value);
    }

    T mean() const {
        T sum = T();
        for (const auto& v : values_) {
            sum = sum + v;
        }
        return sum / values_.size();
    }

    size_t count() const { return values_.size(); }

private:
    std::vector<T> values_;
};

// Specialization for std::string (different behavior)
template<>
class Statistics<std::string> {
public:
    void add(const std::string& value) {
        values_.push_back(value);
        total_length_ += value.length();
    }

    double mean() const {
        return static_cast<double>(total_length_) / values_.size();
    }

    size_t count() const { return values_.size(); }

    std::string concatenated() const {
        std::string result;
        for (const auto& v : values_) {
            result += v + " ";
        }
        return result;
    }

private:
    std::vector<std::string> values_;
    size_t total_length_ = 0;
};

void demo_template_specialization() {
    std::cout << "\n=== Template Specialization ===\n";

    // Generic version for numbers
    Statistics<double> num_stats;
    num_stats.add(10.5);
    num_stats.add(20.3);
    num_stats.add(15.7);
    std::cout << "Number stats - Mean: " << num_stats.mean()
              << ", Count: " << num_stats.count() << "\n";

    // Specialized version for strings
    Statistics<std::string> str_stats;
    str_stats.add("Hello");
    str_stats.add("World");
    str_stats.add("Templates");
    std::cout << "String stats - Mean length: " << str_stats.mean()
              << ", Count: " << str_stats.count() << "\n";
    std::cout << "Concatenated: " << str_stats.concatenated() << "\n";
}

// ============================================================================
// Variadic Templates
// ============================================================================

// Print function for any number of arguments
void print() {
    std::cout << "\n";
}

template<typename T, typename... Args>
void print(T first, Args... rest) {
    std::cout << first << " ";
    print(rest...);  // Recursive call
}

// Generic publisher that can publish to multiple topics
template<typename MessageT>
class MultiPublisher {
public:
    template<typename... Topics>
    MultiPublisher(Topics... topics) {
        (publishers_.emplace_back(std::make_unique<Publisher<MessageT>>(topics)), ...);
    }

    void publish_all(const MessageT& msg) {
        for (auto& pub : publishers_) {
            pub->publish(msg);
        }
    }

private:
    std::vector<std::unique_ptr<Publisher<MessageT>>> publishers_;
};

void demo_variadic_templates() {
    std::cout << "\n=== Variadic Templates ===\n";

    // Print any number of arguments
    print("Variadic", "templates", "are", "powerful!");
    print(1, 2.5, "mixed", 'x');

    // Multi-publisher
    MultiPublisher<StringMsg> multi_pub("topic1", "topic2", "topic3");
    multi_pub.publish_all({"Broadcasting to all topics"});
}

// ============================================================================
// Template Metaprogramming Example
// ============================================================================

// Compile-time factorial
template<int N>
struct Factorial {
    static constexpr int value = N * Factorial<N - 1>::value;
};

template<>
struct Factorial<0> {
    static constexpr int value = 1;
};

// Type traits example
template<typename T>
struct is_pointer {
    static constexpr bool value = false;
};

template<typename T>
struct is_pointer<T*> {
    static constexpr bool value = true;
};

void demo_metaprogramming() {
    std::cout << "\n=== Template Metaprogramming ===\n";

    // Compile-time computation
    std::cout << "Factorial<5> = " << Factorial<5>::value << "\n";
    std::cout << "Factorial<10> = " << Factorial<10>::value << "\n";

    // Type traits
    std::cout << "is_pointer<int>: " << is_pointer<int>::value << "\n";
    std::cout << "is_pointer<int*>: " << is_pointer<int*>::value << "\n";
}

// ============================================================================
// ROS2-Style Template Usage
// ============================================================================

/**
 * @brief Generic node template (simplified ROS2 style)
 */
template<typename MessageT>
class GenericNode {
public:
    GenericNode(const std::string& node_name,
                const std::string& topic,
                std::function<MessageT()> generator)
        : name_(node_name), publisher_(topic), generator_(generator) {
        std::cout << "Node '" << name_ << "' created\n";
    }

    void publish_once() {
        MessageT msg = generator_();
        publisher_.publish(msg);
    }

    void subscribe(typename Publisher<MessageT>::Callback callback) {
        publisher_.subscribe(callback);
    }

private:
    std::string name_;
    Publisher<MessageT> publisher_;
    std::function<MessageT()> generator_;
};

void demo_ros2_style_templates() {
    std::cout << "\n=== ROS2-Style Template Usage ===\n";

    // Sensor node
    GenericNode<SensorMsg> sensor_node(
        "sensor_node",
        "sensor_data",
        []() { return SensorMsg{"temp1", 23.5, true}; }
    );

    sensor_node.subscribe([](const SensorMsg& msg) {
        std::cout << "  Monitor received: " << msg.to_string() << "\n";
    });

    sensor_node.publish_once();

    // String node
    GenericNode<StringMsg> logger_node(
        "logger_node",
        "logs",
        []() { return StringMsg{"Log entry"}; }
    );

    logger_node.subscribe([](const StringMsg& msg) {
        std::cout << "  Logger: " << msg.data << "\n";
    });

    logger_node.publish_once();
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "========================================\n";
    std::cout << "Template Publisher Demonstration\n";
    std::cout << "========================================\n";

    demo_function_templates();
    demo_generic_publisher();
    demo_circular_buffer();
    demo_template_specialization();
    demo_variadic_templates();
    demo_metaprogramming();
    demo_ros2_style_templates();

    std::cout << "\n========================================\n";
    std::cout << "All demonstrations complete!\n";
    std::cout << "========================================\n";

    return 0;
}

/*
Key Takeaways:

1. Function Templates:
   - template<typename T> T func(T a, T b)
   - Type automatically deduced

2. Class Templates:
   - template<typename T> class MyClass
   - Used with type parameter: MyClass<int>

3. Template Specialization:
   - Provide different implementation for specific types
   - template<> class MyClass<SpecialType>

4. Non-type Parameters:
   - template<typename T, size_t N>
   - Compile-time constants

5. Variadic Templates:
   - template<typename... Args>
   - Variable number of template parameters

6. ROS2 Usage:
   - Publishers/Subscribers are templates
   - Message types as template parameters
   - Generic node implementations

7. Benefits:
   - Code reuse
   - Type safety
   - No runtime overhead
   - Compile-time optimization
*/
