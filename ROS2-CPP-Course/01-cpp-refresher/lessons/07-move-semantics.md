# Lesson 7: Move Semantics

## Learning Objectives

- Understand lvalues vs rvalues
- Master move constructors and move assignment
- Use `std::move` correctly
- Apply move semantics in ROS2 for efficiency

## The Problem: Expensive Copying

```cpp
std::vector<int> create_large_vector() {
    std::vector<int> v(1000000, 42);  // 1 million elements
    return v;  // Copies all data? (Before C++11: yes!)
}

std::vector<int> data = create_large_vector();  // Expensive copy?
```

**Move semantics** solves this by **transferring ownership** instead of copying.

## Lvalues vs Rvalues

### Simplified Explanation

- **Lvalue**: Has a name, persists beyond expression
- **Rvalue**: Temporary, no name

```cpp
int x = 5;        // x is lvalue
int y = x + 1;    // x+1 is rvalue (temporary)

std::string s1 = "hello";
std::string s2 = s1;           // s1 is lvalue (copy)
std::string s3 = "world";      // "world" is rvalue (move)
std::string s4 = s1 + s2;      // s1+s2 is rvalue (move)
```

## Move Constructor and Move Assignment

```cpp
class Data {
public:
    // Regular constructor
    Data(size_t size) : size_(size), data_(new int[size]) {
        std::cout << "Constructed\n";
    }

    // Copy constructor (expensive!)
    Data(const Data& other) : size_(other.size_), data_(new int[size_]) {
        std::copy(other.data_, other.data_ + size_, data_);
        std::cout << "Copied (expensive!)\n";
    }

    // Move constructor (cheap!)
    Data(Data&& other) noexcept
        : size_(other.size_), data_(other.data_) {
        other.size_ = 0;
        other.data_ = nullptr;  // Take ownership, reset source
        std::cout << "Moved (cheap!)\n";
    }

    // Copy assignment
    Data& operator=(const Data& other) {
        if (this != &other) {
            delete[] data_;
            size_ = other.size_;
            data_ = new int[size_];
            std::copy(other.data_, other.data_ + size_, data_);
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
        }
        return *this;
    }

    ~Data() {
        delete[] data_;
    }

private:
    size_t size_;
    int* data_;
};
```

## std::move

`std::move` casts an lvalue to an rvalue reference, enabling move semantics:

```cpp
Data d1(1000);
Data d2 = d1;              // Copy (d1 is lvalue)
Data d3 = std::move(d1);   // Move (d1 now in valid but unspecified state)
// Don't use d1 after std::move!
```

## ROS2 Message Publishing (Move)

### Efficient Publishing

```cpp
class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node("publisher") {
        publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() { this->publish(); }
        );
    }

private:
    void publish() {
        // Create message
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Large message data...";

        // Move (zero-copy, efficient!)
        publisher_->publish(std::move(msg));
        // msg is now nullptr, don't use it!
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### Copy vs Move

```cpp
// BAD: Copy (creates duplicate)
std_msgs::msg::String msg;
msg.data = "data";
publisher->publish(msg);  // Copies msg

// GOOD: Move (transfers ownership)
auto msg = std::make_unique<std_msgs::msg::String>();
msg->data = "data";
publisher->publish(std::move(msg));  // Moves, no copy!
```

## Return Value Optimization (RVO)

Compilers automatically apply move semantics when returning:

```cpp
std::vector<int> create_vector() {
    std::vector<int> v = {1, 2, 3, 4, 5};
    return v;  // Automatically moved (or RVO'd)!
}

std::vector<int> data = create_vector();  // No copy!
```

## Practical Examples

### Transferring Unique Pointer

```cpp
std::unique_ptr<Sensor> sensor = std::make_unique<Sensor>("GPS");

// Transfer ownership
std::unique_ptr<Sensor> another = std::move(sensor);
// sensor is now nullptr!

if (!sensor) {
    std::cout << "sensor is null\n";  // This executes
}
```

### Moving into Container

```cpp
std::vector<std::unique_ptr<Sensor>> sensors;

auto gps = std::make_unique<Sensor>("GPS");
auto imu = std::make_unique<Sensor>("IMU");

sensors.push_back(std::move(gps));  // Move into vector
sensors.push_back(std::move(imu));
// gps and imu are now nullptr
```

### Swapping

```cpp
std::string a = "first";
std::string b = "second";

std::swap(a, b);  // Uses move semantics internally (efficient)
```

## When to Use std::move

### DO ✓
```cpp
// Transferring ownership
auto ptr2 = std::move(ptr1);

// Publishing ROS2 messages
publisher->publish(std::move(msg));

// Moving into containers
vec.push_back(std::move(obj));

// Returning local unique_ptr
return std::move(local_ptr);  // (Actually optional due to RVO)
```

### DON'T ✗
```cpp
// Don't move const objects
const Data d;
Data d2 = std::move(d);  // Calls copy, not move!

// Don't use moved-from object
Data d1;
Data d2 = std::move(d1);
d1.use();  // DANGER! d1 is in unspecified state

// Don't move when you need the original
Data important_data;
process(std::move(important_data));  // Can't use important_data anymore!
```

## Summary

**Key Takeaways:**
- Move semantics transfer ownership instead of copying
- Use `std::move` to enable moving from lvalues
- ROS2 message publishing uses move for efficiency
- Don't use objects after `std::move`
- Modern C++ automatically moves in many cases (RVO)

**Critical for ROS2:**
- `publisher->publish(std::move(msg))` is the standard pattern
- Enables zero-copy message passing
- Essential for performance with large messages

## What's Next?

- **Next Lesson**: [STL Essentials](08-stl-essentials.md)
- **Practice**: Implement move semantics in custom classes
