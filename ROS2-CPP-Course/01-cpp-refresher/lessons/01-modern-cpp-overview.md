# Lesson 1: Modern C++ Overview

## Learning Objectives

- Understand the evolution of C++ standards
- Identify key features introduced in C++11/14/17/20
- Recognize why modern C++ matters for ROS2
- Know which features you'll use most

## Introduction

C++ has evolved significantly since its creation. ROS2 uses **modern C++** (C++17 standard), which is vastly different from "old" C++ (pre-C++11). This lesson introduces the modern features you'll encounter.

## C++ Standards Timeline

| Year | Version | Key Features | Status in ROS2 |
|------|---------|-------------|----------------|
| 1998 | C++98 | Original standard, STL | Legacy |
| 2003 | C++03 | Bug fixes | Legacy |
| 2011 | C++11 | **Modern C++ begins!** Smart pointers, lambdas, auto | Used heavily |
| 2014 | C++14 | Refinements to C++11 | Used |
| 2017 | C++17 | std::optional, filesystem | **ROS2 standard** |
| 2020 | C++20 | Concepts, ranges, coroutines | Future ROS2 |

**ROS2 Humble uses C++17** as the minimum standard.

## Why Modern C++ for ROS2?

### 1. Safety
Modern C++ reduces errors through:
- Smart pointers (automatic memory management)
- Type safety (auto, strong typing)
- RAII (Resource Acquisition Is Initialization)

### 2. Performance
- Move semantics (zero-copy operations)
- Better compiler optimizations
- Efficient generic programming (templates)

### 3. Expressiveness
- Lambda functions (concise callbacks)
- Auto type deduction (less boilerplate)
- Range-based for loops (cleaner iteration)

### 4. Standard Library
- Rich containers (vector, map, etc.)
- Algorithms (sort, find, etc.)
- Utilities (optional, variant, etc.)

##Key Modern C++ Features

### Feature 1: Auto Type Deduction

**Before C++11:**
```cpp
std::map<std::string, std::vector<int>>::iterator it = myMap.begin();
```

**With auto:**
```cpp
auto it = myMap.begin();  // Type deduced automatically
```

**Why it matters in ROS2:**
```cpp
// ROS2 publisher creation - long type!
auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
// vs.
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = ...;
```

### Feature 2: Smart Pointers

**Old C++ (raw pointers - dangerous!):**
```cpp
MyClass* ptr = new MyClass();
// ... use ptr ...
delete ptr;  // Easy to forget! Memory leak!
```

**Modern C++ (smart pointers - safe!):**
```cpp
auto ptr = std::make_unique<MyClass>();
// Automatically deleted when ptr goes out of scope!
```

**In ROS2:**
```cpp
// ROS2 nodes use shared_ptr extensively
auto node = std::make_shared<rclcpp::Node>("my_node");
```

### Feature 3: Lambda Functions

**Old C++ (function pointers or functors - verbose):**
```cpp
void callback(int x) {
    std::cout << x << std::endl;
}
// Pass callback function
```

**Modern C++ (lambdas - concise):**
```cpp
auto callback = [](int x) {
    std::cout << x << std::endl;
};
```

**In ROS2 (callbacks everywhere!):**
```cpp
// Timer callback
timer_ = node->create_wall_timer(
    std::chrono::seconds(1),
    []() {  // Lambda!
        RCLCPP_INFO(node->get_logger(), "Timer fired!");
    }
);
```

### Feature 4: Range-Based For Loops

**Old C++ (iterator-based):**
```cpp
for (std::vector<int>::iterator it = vec.begin(); it != vec.end(); ++it) {
    std::cout << *it << std::endl;
}
```

**Modern C++ (range-based):**
```cpp
for (const auto& element : vec) {
    std::cout << element << std::endl;
}
```

**In ROS2:**
```cpp
// Iterate over parameters
for (const auto& param : node->get_parameters()) {
    RCLCPP_INFO(node->get_logger(), "Param: %s", param.get_name().c_str());
}
```

### Feature 5: Nullptr

**Old C++ (NULL or 0 - ambiguous):**
```cpp
MyClass* ptr = NULL;  // Actually 0, can cause issues
```

**Modern C++ (nullptr - type-safe):**
```cpp
MyClass* ptr = nullptr;  // Proper null pointer
```

### Feature 6: Uniform Initialization

**Modern C++ (brace initialization):**
```cpp
std::vector<int> vec{1, 2, 3, 4, 5};
MyClass obj{param1, param2};
```

**Prevents narrowing conversions:**
```cpp
int x = 7.9;      // OK, but truncates to 7
int y{7.9};       // ERROR! Prevents accidental data loss
```

### Feature 7: Move Semantics

**Copy (expensive for large data):**
```cpp
std::vector<int> vec1 = getLargeVector();  // Copies all elements
```

**Move (efficient):**
```cpp
std::vector<int> vec1 = std::move(getLargeVector());  // Transfers ownership
```

**In ROS2 (message passing):**
```cpp
auto msg = std::make_unique<std_msgs::msg::String>();
msg->data = "Hello";
publisher->publish(std::move(msg));  // Moves, doesn't copy!
```

### Feature 8: Structured Bindings (C++17)

**Unpack tuples/pairs easily:**
```cpp
std::map<std::string, int> myMap = {{"a", 1}, {"b", 2}};

for (const auto& [key, value] : myMap) {
    std::cout << key << ": " << value << std::endl;
}
```

### Feature 9: std::optional (C++17)

**Represent optional values safely:**
```cpp
std::optional<int> maybe_value = findValue();

if (maybe_value.has_value()) {
    std::cout << "Found: " << maybe_value.value() << std::endl;
} else {
    std::cout << "Not found" << std::endl;
}
```

**In ROS2:**
```cpp
// Parameter might not exist
auto param = node->get_parameter_or<int>("my_param", 42);
```

### Feature 10: constexpr

**Compile-time evaluation:**
```cpp
constexpr int factorial(int n) {
    return n <= 1 ? 1 : n * factorial(n - 1);
}

constexpr int result = factorial(5);  // Computed at compile time!
```

## Modern C++ in ROS2 Context

Let's see a complete example showing modern C++ features in ROS2:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class ModernPublisher : public rclcpp::Node {
public:
    ModernPublisher() : Node("modern_publisher") {  // Uniform initialization
        // Auto type deduction
        auto qos = rclcpp::QoS(10);

        // Smart pointer (shared_ptr)
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos);

        // Lambda function for callback
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {  // Capture 'this' by value
                this->publish_message();
            }
        );
    }

private:
    void publish_message() {
        // Make unique_ptr message
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello from modern C++!";

        // Move semantics - efficient!
        publisher_->publish(std::move(msg));
    }

    // Smart pointers as members
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Make shared node
    auto node = std::make_shared<ModernPublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

**Modern features used:**
- ✓ Auto type deduction
- ✓ Smart pointers (`shared_ptr`, `unique_ptr`)
- ✓ Lambda functions
- ✓ Move semantics
- ✓ Uniform initialization

## Common Patterns in ROS2

### Pattern 1: Factory Functions
```cpp
// Modern C++ prefers make_unique/make_shared
auto node = std::make_shared<MyNode>();
auto msg = std::make_unique<MyMessage>();
```

### Pattern 2: Lambda Callbacks
```cpp
// Timers, subscriptions all use lambdas
subscription_ = create_subscription<MsgType>(
    "topic",
    10,
    [this](const MsgType::SharedPtr msg) {
        // Handle message
    }
);
```

### Pattern 3: Move-Based Publishing
```cpp
// Efficient zero-copy publishing
auto msg = std::make_unique<MsgType>();
publisher_->publish(std::move(msg));
```

## Python vs Modern C++ Comparison

| Concept | Python | Modern C++ |
|---------|--------|-----------|
| Variable declaration | `x = 5` | `auto x = 5;` or `int x = 5;` |
| Function | `def func(x):` | `auto func(int x) { }` |
| List | `mylist = [1, 2, 3]` | `std::vector<int> vec{1, 2, 3};` |
| Dictionary | `mydict = {"a": 1}` | `std::map<string, int> map{{"a", 1}};` |
| Lambda | `lambda x: x * 2` | `[](int x) { return x * 2; }` |
| Loop | `for item in items:` | `for (const auto& item : items) {` |
| None/null | `None` | `nullptr` or `std::nullopt` |

## Common Pitfalls to Avoid

### 1. Forgetting const References
```cpp
// Bad - copies every element!
for (auto element : vec) { }

// Good - reference, no copy
for (const auto& element : vec) { }
```

### 2. Misusing auto
```cpp
// Bad - loses const
const int x = 5;
auto y = x;  // y is 'int', not 'const int'

// Good - preserves const
auto y = std::as_const(x);  // or: const auto y = x;
```

### 3. Dangling References
```cpp
// Bad - returns reference to local variable!
const std::string& getBadString() {
    std::string local = "bad";
    return local;  // DANGLING!
}

// Good - return by value (move semantics makes it efficient)
std::string getGoodString() {
    return "good";  // Moved, not copied!
}
```

## Best Practices

1. **Use `auto`** for complex types, clarity for simple types
2. **Prefer `make_unique`/`make_shared`** over `new`
3. **Use lambda captures carefully** (avoid capturing dangling references)
4. **Pass by `const&`** for read-only, by value for ownership transfer
5. **Use `std::move`** when transferring ownership
6. **Enable warnings** (`-Wall -Wextra`) and treat them as errors

## Summary

**Key Takeaways:**
- Modern C++ (C++11/14/17/20) is much safer and cleaner than old C++
- ROS2 uses C++17 standard with modern features throughout
- Smart pointers, lambdas, and move semantics are essential for ROS2
- Auto, range-based for, and uniform initialization reduce boilerplate
- Modern C++ enables efficient, safe robotics code

**Most Important for ROS2:**
1. Smart pointers (`unique_ptr`, `shared_ptr`)
2. Lambda functions (callbacks)
3. Move semantics (efficient messaging)
4. Auto (less typing)

## What's Next?

- **Next Lesson**: [OOP Essentials](02-oop-essentials.md) - Classes and inheritance in C++
- **Practice**: Try writing simple programs using `auto`, smart pointers, and lambdas
- **Reference**: [cppreference.com](https://en.cppreference.com/) for detailed documentation

---

**You're on your way to modern C++!** The features covered here will appear constantly in ROS2 code.
