/**
 * @file lambda_callbacks.cpp
 * @brief Comprehensive demonstration of lambda functions and callbacks
 *
 * This example demonstrates:
 * - Lambda syntax and captures
 * - Lambda callbacks in event systems
 * - Simulated ROS2-style callbacks
 * - Practical lambda patterns
 *
 * Compile: g++ -std=c++17 -Wall lambda_callbacks.cpp -o lambda_callbacks
 * Run: ./lambda_callbacks
 */

#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <memory>
#include <chrono>
#include <string>

// ============================================================================
// Simple Event System
// ============================================================================

/**
 * @brief Simple event emitter that calls registered callbacks
 */
class EventEmitter {
public:
    using Callback = std::function<void(int)>;

    void on_event(Callback callback) {
        callbacks_.push_back(callback);
    }

    void emit(int value) {
        std::cout << "\n[Event] Emitting value: " << value << "\n";
        for (const auto& callback : callbacks_) {
            callback(value);
        }
    }

private:
    std::vector<Callback> callbacks_;
};

// ============================================================================
// Lambda Syntax Demonstrations
// ============================================================================

void demo_basic_lambdas() {
    std::cout << "\n=== Basic Lambda Syntax ===\n";

    // 1. No parameters, no capture
    auto greet = []() {
        std::cout << "Hello from lambda!\n";
    };
    greet();

    // 2. With parameters
    auto add = [](int a, int b) {
        return a + b;
    };
    std::cout << "5 + 3 = " << add(5, 3) << "\n";

    // 3. With explicit return type
    auto divide = [](double a, double b) -> double {
        return a / b;
    };
    std::cout << "10 / 3 = " << divide(10.0, 3.0) << "\n";

    // 4. Generic lambda (C++14)
    auto print = [](const auto& value) {
        std::cout << "Value: " << value << "\n";
    };
    print(42);
    print(3.14);
    print("Hello");
}

// ============================================================================
// Lambda Captures
// ============================================================================

void demo_lambda_captures() {
    std::cout << "\n=== Lambda Captures ===\n";

    int x = 10;
    int y = 20;

    // Capture by value
    auto capture_value = [x, y]() {
        std::cout << "Captured by value: x=" << x << ", y=" << y << "\n";
    };

    // Capture by reference
    auto capture_ref = [&x, &y]() {
        x += 10;
        y += 10;
        std::cout << "Modified via reference: x=" << x << ", y=" << y << "\n";
    };

    // Capture all by value
    auto capture_all_value = [=]() {
        std::cout << "All by value: x=" << x << ", y=" << y << "\n";
    };

    // Capture all by reference
    auto capture_all_ref = [&]() {
        std::cout << "All by reference: x=" << x << ", y=" << y << "\n";
    };

    // Mixed capture
    auto mixed = [x, &y]() {
        std::cout << "Mixed: x=" << x << " (value), y=" << y << " (reference)\n";
    };

    capture_value();
    capture_ref();
    capture_all_value();
    capture_all_ref();
    mixed();

    std::cout << "After all captures: x=" << x << ", y=" << y << "\n";
}

// ============================================================================
// Lambda with Event System
// ============================================================================

void demo_event_callbacks() {
    std::cout << "\n=== Event System with Lambdas ===\n";

    EventEmitter emitter;
    int counter = 0;

    // Register multiple callbacks using lambdas

    // Callback 1: Simple printing
    emitter.on_event([](int value) {
        std::cout << "  Callback 1: Received " << value << "\n";
    });

    // Callback 2: Capture external variable by reference
    emitter.on_event([&counter](int value) {
        counter += value;
        std::cout << "  Callback 2: Counter is now " << counter << "\n";
    });

    // Callback 3: Conditional logic
    emitter.on_event([](int value) {
        if (value > 50) {
            std::cout << "  Callback 3: HIGH value detected!\n";
        } else {
            std::cout << "  Callback 3: Normal value\n";
        }
    });

    // Emit events
    emitter.emit(25);
    emitter.emit(75);
    emitter.emit(100);

    std::cout << "Final counter value: " << counter << "\n";
}

// ============================================================================
// Simulated ROS2-Style Publisher/Subscriber
// ============================================================================

/**
 * @brief Simulated message type
 */
struct Message {
    std::string data;
    int sequence;
};

/**
 * @brief Simulated subscriber with lambda callback
 */
class Subscriber {
public:
    using Callback = std::function<void(const Message&)>;

    Subscriber(const std::string& topic, Callback callback)
        : topic_(topic), callback_(callback) {}

    void receive(const Message& msg) {
        std::cout << "[" << topic_ << "] ";
        callback_(msg);
    }

private:
    std::string topic_;
    Callback callback_;
};

/**
 * @brief Simulated publisher
 */
class Publisher {
public:
    Publisher(const std::string& topic) : topic_(topic), sequence_(0) {}

    void publish(const std::string& data) {
        Message msg{data, sequence_++};
        for (auto& sub : subscribers_) {
            sub->receive(msg);
        }
    }

    void subscribe(std::shared_ptr<Subscriber> sub) {
        subscribers_.push_back(sub);
    }

private:
    std::string topic_;
    int sequence_;
    std::vector<std::shared_ptr<Subscriber>> subscribers_;
};

void demo_ros2_style_callbacks() {
    std::cout << "\n=== ROS2-Style Callbacks ===\n";

    Publisher pub("sensor_data");

    // Subscriber 1: Simple lambda callback
    auto sub1 = std::make_shared<Subscriber>(
        "logger",
        [](const Message& msg) {
            std::cout << "Logged: [" << msg.sequence << "] " << msg.data << "\n";
        }
    );

    // Subscriber 2: Lambda with capture for statistics
    int total_messages = 0;
    int total_length = 0;

    auto sub2 = std::make_shared<Subscriber>(
        "stats",
        [&total_messages, &total_length](const Message& msg) {
            total_messages++;
            total_length += msg.data.length();
            double avg = static_cast<double>(total_length) / total_messages;
            std::cout << "Stats: " << total_messages << " messages, "
                      << "avg length: " << avg << "\n";
        }
    );

    // Subscriber 3: Lambda with conditional processing
    auto sub3 = std::make_shared<Subscriber>(
        "filter",
        [](const Message& msg) {
            if (msg.data.find("ERROR") != std::string::npos) {
                std::cout << "!!! ERROR DETECTED in message " << msg.sequence << "\n";
            }
        }
    );

    pub.subscribe(sub1);
    pub.subscribe(sub2);
    pub.subscribe(sub3);

    // Publish messages
    pub.publish("System started");
    pub.publish("Sensor reading: 23.5");
    pub.publish("ERROR: Connection lost");
    pub.publish("Reconnected successfully");
}

// ============================================================================
// Timer Simulation (ROS2-style)
// ============================================================================

/**
 * @brief Simulated timer
 */
class Timer {
public:
    using Callback = std::function<void()>;

    Timer(int interval_ms, Callback callback)
        : interval_ms_(interval_ms), callback_(callback), tick_count_(0) {}

    void tick() {
        tick_count_++;
        std::cout << "[Timer Tick " << tick_count_ << "] ";
        callback_();
    }

    void simulate(int num_ticks) {
        for (int i = 0; i < num_ticks; ++i) {
            tick();
        }
    }

private:
    int interval_ms_;
    Callback callback_;
    int tick_count_;
};

void demo_timer_callbacks() {
    std::cout << "\n=== Timer Callbacks ===\n";

    int counter = 0;

    // Timer with lambda callback that captures counter
    Timer timer(1000, [&counter]() {
        counter++;
        std::cout << "Counter: " << counter << "\n";
    });

    timer.simulate(5);

    std::cout << "Final counter: " << counter << "\n";
}

// ============================================================================
// STL Algorithms with Lambdas
// ============================================================================

struct SensorReading {
    double value;
    bool valid;
    std::string sensor_id;
};

void demo_stl_algorithms() {
    std::cout << "\n=== STL Algorithms with Lambdas ===\n";

    std::vector<SensorReading> readings = {
        {23.5, true, "temp1"},
        {99.9, false, "temp2"},  // Invalid
        {24.1, true, "temp3"},
        {-10.0, false, "temp4"},  // Invalid
        {25.0, true, "temp5"}
    };

    // Filter valid readings using lambda
    std::cout << "Valid readings:\n";
    std::copy_if(
        readings.begin(),
        readings.end(),
        std::ostream_iterator<SensorReading>(std::cout, ""),
        [](const SensorReading& r) { return r.valid; }
    );

    // Find maximum value using lambda
    auto max_it = std::max_element(
        readings.begin(),
        readings.end(),
        [](const SensorReading& a, const SensorReading& b) {
            return a.value < b.value;
        }
    );
    std::cout << "\nMaximum value: " << max_it->value
              << " from " << max_it->sensor_id << "\n";

    // Count valid readings
    int valid_count = std::count_if(
        readings.begin(),
        readings.end(),
        [](const SensorReading& r) { return r.valid; }
    );
    std::cout << "Valid readings count: " << valid_count << "\n";

    // Transform values using lambda
    std::vector<double> celsius_values;
    std::transform(
        readings.begin(),
        readings.end(),
        std::back_inserter(celsius_values),
        [](const SensorReading& r) { return r.value; }
    );

    std::cout << "All values: ";
    for (double v : celsius_values) {
        std::cout << v << " ";
    }
    std::cout << "\n";
}

// Output operator for SensorReading
std::ostream& operator<<(std::ostream& os, const SensorReading& r) {
    os << "  " << r.sensor_id << ": " << r.value
       << (r.valid ? " (valid)" : " (invalid)") << "\n";
    return os;
}

// ============================================================================
// Mutable Lambdas
// ============================================================================

void demo_mutable_lambdas() {
    std::cout << "\n=== Mutable Lambdas ===\n";

    int external = 0;

    // Mutable lambda - can modify captured-by-value variables
    auto counter = [count = 0]() mutable {
        count++;
        std::cout << "Internal count: " << count << "\n";
        return count;
    };

    counter();
    counter();
    counter();
    std::cout << "External variable unchanged: " << external << "\n";

    // Capture by reference doesn't need mutable
    auto ref_counter = [&external]() {
        external++;
        std::cout << "External count: " << external << "\n";
    };

    ref_counter();
    ref_counter();
    ref_counter();
}

// ============================================================================
// Lambda as Factory
// ============================================================================

void demo_lambda_factory() {
    std::cout << "\n=== Lambda as Factory ===\n";

    // Factory that creates counter lambdas
    auto create_counter = [](int start) {
        return [count = start]() mutable {
            return count++;
        };
    };

    auto counter1 = create_counter(0);
    auto counter2 = create_counter(100);

    std::cout << "Counter1: " << counter1() << "\n";  // 0
    std::cout << "Counter1: " << counter1() << "\n";  // 1
    std::cout << "Counter2: " << counter2() << "\n";  // 100
    std::cout << "Counter2: " << counter2() << "\n";  // 101
    std::cout << "Counter1: " << counter1() << "\n";  // 2
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "========================================\n";
    std::cout << "Lambda Functions and Callbacks Demo\n";
    std::cout << "========================================\n";

    demo_basic_lambdas();
    demo_lambda_captures();
    demo_event_callbacks();
    demo_ros2_style_callbacks();
    demo_timer_callbacks();
    demo_stl_algorithms();
    demo_mutable_lambdas();
    demo_lambda_factory();

    std::cout << "\n========================================\n";
    std::cout << "All demonstrations complete!\n";
    std::cout << "========================================\n";

    return 0;
}

/*
Key Takeaways:

1. Lambda Syntax: [capture](params) -> return_type { body }

2. Captures:
   - [=] : Capture all by value
   - [&] : Capture all by reference
   - [x, &y] : Mixed capture

3. Common Patterns:
   - Event callbacks
   - ROS2-style pub/sub callbacks
   - Timer callbacks
   - STL algorithms

4. Mutable lambdas allow modifying captured-by-value variables

5. Lambdas can be used as factories to create functions
*/
