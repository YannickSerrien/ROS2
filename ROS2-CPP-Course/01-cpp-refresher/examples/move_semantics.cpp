/**
 * @file move_semantics.cpp
 * @brief Demonstration of move semantics vs copy semantics
 *
 * This example demonstrates:
 * - Copy vs move constructors
 * - std::move usage
 * - Performance implications
 * - ROS2-style message passing
 * - Return value optimization
 *
 * Compile: g++ -std=c++17 -Wall move_semantics.cpp -o move_semantics
 * Run: ./move_semantics
 */

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <algorithm>

// ============================================================================
// Data Class - Demonstrates Copy vs Move
// ============================================================================

class Data {
public:
    // Constructor
    Data(size_t size = 1000) : size_(size), data_(new int[size]) {
        std::fill_n(data_, size_, 42);
        std::cout << "  [Constructed] Size: " << size_ << "\n";
    }

    // Copy constructor (expensive!)
    Data(const Data& other) : size_(other.size_), data_(new int[size_]) {
        std::copy(other.data_, other.data_ + size_, data_);
        std::cout << "  [COPIED] Size: " << size_ << " (expensive!)\n";
    }

    // Move constructor (cheap!)
    Data(Data&& other) noexcept
        : size_(other.size_), data_(other.data_) {
        other.size_ = 0;
        other.data_ = nullptr;
        std::cout << "  [MOVED] Size: " << size_ << " (cheap!)\n";
    }

    // Copy assignment
    Data& operator=(const Data& other) {
        if (this != &other) {
            delete[] data_;
            size_ = other.size_;
            data_ = new int[size_];
            std::copy(other.data_, other.data_ + size_, data_);
            std::cout << "  [Copy Assignment] Size: " << size_ << "\n";
        }
        return *this;
    }

    // Move assignment
    Data& operator=(Data&& other) noexcept {
        if (this != &other) {
            delete[] data_;
            size_ = other.size_;
            data_ = other.data_;
            other.size_ = 0;
            other.data_ = nullptr;
            std::cout << "  [Move Assignment] Size: " << size_ << "\n";
        }
        return *this;
    }

    // Destructor
    ~Data() {
        if (data_) {
            std::cout << "  [Destroyed] Size: " << size_ << "\n";
        } else {
            std::cout << "  [Destroyed] Empty (moved-from)\n";
        }
        delete[] data_;
    }

    size_t size() const { return size_; }

private:
    size_t size_;
    int* data_;
};

// ============================================================================
// Copy vs Move Comparison
// ============================================================================

void demo_copy_vs_move() {
    std::cout << "\n=== Copy vs Move ===\n";

    std::cout << "\n1. Copy (lvalue):\n";
    Data d1(1000);
    Data d2 = d1;  // Copy constructor called

    std::cout << "\n2. Move (rvalue with std::move):\n";
    Data d3(1000);
    Data d4 = std::move(d3);  // Move constructor called
    std::cout << "d3 size after move: " << d3.size() << " (empty)\n";

    std::cout << "\n3. Temporary (automatic move):\n";
    Data d5 = Data(1000);  // Move (or RVO), not copy!
}

// ============================================================================
// Function Return Values
// ============================================================================

Data create_large_data() {
    std::cout << "  Creating data in function...\n";
    Data local(10000);
    return local;  // Return value optimization (RVO) or move
}

void demo_return_values() {
    std::cout << "\n=== Function Return Values ===\n";

    std::cout << "\nReturning from function:\n";
    Data result = create_large_data();
    std::cout << "Result size: " << result.size() << "\n";
    // Note: Likely no copy or move due to RVO (Return Value Optimization)
}

// ============================================================================
// Vector and Containers
// ============================================================================

void demo_vector_move() {
    std::cout << "\n=== Vector with Move Semantics ===\n";

    std::vector<Data> vec;
    vec.reserve(3);  // Avoid reallocation

    std::cout << "\n1. Push lvalue (copy):\n";
    Data d1(100);
    vec.push_back(d1);  // Copy

    std::cout << "\n2. Push rvalue with std::move (move):\n";
    Data d2(100);
    vec.push_back(std::move(d2));  // Move

    std::cout << "\n3. Push temporary (move):\n";
    vec.push_back(Data(100));  // Move (temporary)

    std::cout << "\nVector now has " << vec.size() << " elements\n";
}

// ============================================================================
// Perfect Forwarding Simulation
// ============================================================================

template<typename T>
void process_value(T&& value) {
    // Forward the value preserving its value category
    Data result = std::forward<T>(value);
}

void demo_forwarding() {
    std::cout << "\n=== Perfect Forwarding ===\n";

    Data d1(100);

    std::cout << "\nForwarding lvalue (copy):\n";
    process_value(d1);

    std::cout << "\nForwarding rvalue (move):\n";
    process_value(std::move(d1));

    std::cout << "\nForwarding temporary (move):\n";
    process_value(Data(100));
}

// ============================================================================
// ROS2-Style Message Publishing
// ============================================================================

struct Message {
    std::string data;
    std::vector<double> readings;

    Message(const std::string& d, size_t num_readings = 1000)
        : data(d), readings(num_readings, 42.0) {
        std::cout << "  [Message Created] " << num_readings << " readings\n";
    }

    Message(const Message& other)
        : data(other.data), readings(other.readings) {
        std::cout << "  [Message COPIED] " << readings.size() << " readings\n";
    }

    Message(Message&& other) noexcept
        : data(std::move(other.data)), readings(std::move(other.readings)) {
        std::cout << "  [Message MOVED] " << readings.size() << " readings\n";
    }
};

class Publisher {
public:
    // Publish by copy (inefficient)
    void publish_copy(const Message& msg) {
        std::cout << "\nPublishing by copy:\n";
        Message local_copy = msg;  // Copy!
        // Simulate sending...
    }

    // Publish by move (efficient)
    void publish_move(Message&& msg) {
        std::cout << "\nPublishing by move:\n";
        Message local_msg = std::move(msg);  // Move!
        // Simulate sending...
    }

    // Publish with unique_ptr (ROS2 pattern)
    void publish_unique(std::unique_ptr<Message> msg) {
        std::cout << "\nPublishing with unique_ptr:\n";
        // Ownership transferred, no copy!
        // Simulate sending...
    }
};

void demo_ros2_publishing() {
    std::cout << "\n=== ROS2-Style Publishing ===\n";

    Publisher pub;

    // Method 1: Copy (inefficient)
    Message msg1("sensor_data", 10000);
    pub.publish_copy(msg1);
    // msg1 still usable

    // Method 2: Move (better)
    Message msg2("sensor_data", 10000);
    pub.publish_move(std::move(msg2));
    // msg2 now in moved-from state, shouldn't use

    // Method 3: unique_ptr (best for ROS2)
    auto msg3 = std::make_unique<Message>("sensor_data", 10000);
    pub.publish_unique(std::move(msg3));
    // msg3 is now nullptr
}

// ============================================================================
// Swap Operations
// ============================================================================

void demo_swap() {
    std::cout << "\n=== Swap with Move Semantics ===\n";

    Data d1(100);
    Data d2(200);

    std::cout << "\nBefore swap: d1=" << d1.size() << ", d2=" << d2.size() << "\n";

    std::cout << "\nSwapping (uses move internally):\n";
    std::swap(d1, d2);  // Efficient with move semantics

    std::cout << "After swap: d1=" << d1.size() << ", d2=" << d2.size() << "\n";
}

// ============================================================================
// Performance Comparison
// ============================================================================

void demo_performance() {
    std::cout << "\n=== Performance Comparison ===\n";

    const size_t NUM_ELEMENTS = 1000;
    const size_t DATA_SIZE = 10000;

    // Copy-based approach
    {
        std::cout << "\nCopy-based approach:\n";
        auto start = std::chrono::high_resolution_clock::now();

        std::vector<Data> vec;
        vec.reserve(NUM_ELEMENTS);
        for (size_t i = 0; i < NUM_ELEMENTS; ++i) {
            Data d(DATA_SIZE);
            vec.push_back(d);  // Copy!
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Time: " << duration.count() << "ms\n";
    }

    // Move-based approach
    {
        std::cout << "\nMove-based approach:\n";
        auto start = std::chrono::high_resolution_clock::now();

        std::vector<Data> vec;
        vec.reserve(NUM_ELEMENTS);
        for (size_t i = 0; i < NUM_ELEMENTS; ++i) {
            vec.push_back(Data(DATA_SIZE));  // Move!
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Time: " << duration.count() << "ms\n";
    }
}

// ============================================================================
// Common Pitfalls
// ============================================================================

void demo_pitfalls() {
    std::cout << "\n=== Common Pitfalls ===\n";

    // Pitfall 1: Using moved-from object
    std::cout << "\n1. Don't use after std::move:\n";
    Data d1(100);
    Data d2 = std::move(d1);
    std::cout << "d1 size after move: " << d1.size() << " (in valid but unspecified state)\n";
    // d1.some_method();  // DANGER! Don't use moved-from objects

    // Pitfall 2: Moving const objects (becomes copy!)
    std::cout << "\n2. Can't move const objects:\n";
    const Data d3(100);
    Data d4 = std::move(d3);  // Calls copy constructor, not move!

    // Pitfall 3: Unnecessary std::move on return
    // (RVO handles this automatically)
    std::cout << "\n3. RVO vs explicit move on return:\n";
    auto get_data = []() {
        Data local(100);
        return local;  // RVO - don't need std::move here!
        // return std::move(local);  // Actually prevents RVO!
    };
    Data result = get_data();
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "========================================\n";
    std::cout << "Move Semantics Demonstration\n";
    std::cout << "========================================\n";

    demo_copy_vs_move();
    demo_return_values();
    demo_vector_move();
    demo_forwarding();
    demo_ros2_publishing();
    demo_swap();
    demo_performance();
    demo_pitfalls();

    std::cout << "\n========================================\n";
    std::cout << "All demonstrations complete!\n";
    std::cout << "========================================\n";

    return 0;
}

/*
Key Takeaways:

1. Move semantics transfer ownership instead of copying data

2. std::move casts to rvalue reference, enabling move

3. Move constructor/assignment should:
   - Transfer resources from source
   - Leave source in valid but unspecified state
   - Be marked noexcept

4. ROS2 pattern: publish(std::move(msg))
   - Zero-copy message passing
   - Essential for performance

5. Automatic moves:
   - Returning temporaries
   - RVO (Return Value Optimization)
   - push_back(temporary)

6. Don't:
   - Use objects after std::move
   - Move const objects (becomes copy)
   - Unnecessarily std::move on return
*/
