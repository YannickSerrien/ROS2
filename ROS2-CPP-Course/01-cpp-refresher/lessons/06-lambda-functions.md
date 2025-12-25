# Lesson 6: Lambda Functions

## Learning Objectives

- Understand lambda function syntax
- Master lambda captures (by value and reference)
- Use lambdas in ROS2 callbacks
- Know when to use lambdas vs regular functions
- Avoid common lambda pitfalls

## Introduction

**Lambda functions** (anonymous functions) are one of the most powerful features of modern C++. In ROS2, they're used everywhere for callbacks - timers, subscriptions, services, and actions. Understanding lambdas is essential for ROS2 development!

## What is a Lambda?

A lambda is an **anonymous function** defined inline. Think of it as a function without a name.

### Python Comparison

```python
# Python lambda
square = lambda x: x * x
result = square(5)  # 25

# Or inline
numbers = [1, 2, 3, 4]
squared = map(lambda x: x * x, numbers)
```

### C++ Lambda

```cpp
// C++ lambda
auto square = [](int x) { return x * x; };
int result = square(5);  // 25

// Or inline
std::vector<int> numbers = {1, 2, 3, 4};
std::transform(numbers.begin(), numbers.end(), numbers.begin(),
    [](int x) { return x * x; });
```

## Lambda Syntax

### Basic Structure

```cpp
[capture](parameters) -> return_type { body }
```

- **`[capture]`**: What variables from outer scope to capture
- **`(parameters)`**: Function parameters
- **`-> return_type`**: Return type (usually optional, auto-deduced)
- **`{ body }`**: Function body

### Simple Examples

```cpp
// No parameters, no capture
auto greet = []() {
    std::cout << "Hello!\n";
};
greet();  // Prints "Hello!"

// With parameters
auto add = [](int a, int b) {
    return a + b;
};
int sum = add(3, 4);  // 7

// Explicit return type
auto divide = [](double a, double b) -> double {
    return a / b;
};

// Implicit return type (auto-deduced)
auto multiply = [](int a, int b) {
    return a * b;  // Returns int
};
```

## Lambda Captures

Captures allow lambdas to access variables from the enclosing scope.

### Capture Modes

| Syntax | Meaning |
|--------|---------|
| `[]` | Capture nothing |
| `[=]` | Capture all by value (copy) |
| `[&]` | Capture all by reference |
| `[x]` | Capture `x` by value |
| `[&x]` | Capture `x` by reference |
| `[=, &x]` | Capture all by value, except `x` by reference |
| `[&, x]` | Capture all by reference, except `x` by value |
| `[this]` | Capture `this` pointer (in member functions) |

### Capture by Value

```cpp
int x = 10;
int y = 20;

// Capture x and y by value (copy)
auto lambda = [x, y]() {
    std::cout << x + y << std::endl;  // 30
};

x = 100;  // Change x
lambda(); // Still prints 30 (captured value doesn't change)
```

### Capture by Reference

```cpp
int count = 0;

// Capture count by reference
auto increment = [&count]() {
    count++;
};

increment();
increment();
std::cout << count << std::endl;  // 2
```

### Capture All

```cpp
int a = 1, b = 2, c = 3;

// Capture all by value
auto lambda1 = [=]() {
    return a + b + c;  // 6
};

// Capture all by reference
auto lambda2 = [&]() {
    a++;
    b++;
    c++;
};

lambda2();
std::cout << a << b << c << std::endl;  // "234"
```

### Mixed Captures

```cpp
int x = 10;
int y = 20;

// Capture x by value, y by reference
auto lambda = [x, &y]() {
    y = x + y;  // y modified
    // x = 100;  // ERROR: x is const (captured by value)
};

lambda();
std::cout << y << std::endl;  // 30
```

### Capturing `this` in Member Functions

```cpp
class Counter {
public:
    Counter() : count_(0) {}

    void start_timer() {
        // Capture 'this' to access member variables
        auto callback = [this]() {
            count_++;
            std::cout << "Count: " << count_ << std::endl;
        };

        // Use callback...
    }

private:
    int count_;
};
```

## Lambdas in ROS2

ROS2 uses lambdas extensively for callbacks!

### Timer Callbacks

```cpp
class TimerNode : public rclcpp::Node {
public:
    TimerNode() : Node("timer_node"), count_(0) {
        // Lambda for timer callback
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {  // Capture 'this'
                count_++;
                RCLCPP_INFO(this->get_logger(), "Timer tick: %d", count_);
            }
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};
```

### Subscription Callbacks

```cpp
class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("subscriber_node") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg) {  // Lambda callback
                RCLCPP_INFO(
                    this->get_logger(),
                    "Received: '%s'",
                    msg->data.c_str()
                );
            }
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```

### Service Callbacks

```cpp
class ServiceNode : public rclcpp::Node {
public:
    ServiceNode() : Node("service_node") {
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_flag",
            [this](
                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response
            ) {
                // Lambda service callback
                response->success = true;
                response->message = request->data ? "Set to true" : "Set to false";
                RCLCPP_INFO(this->get_logger(), "Service called");
            }
        );
    }

private:
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};
```

## Common Lambda Patterns

### Pattern 1: Simple Callback

```cpp
// Inline lambda for one-time use
button.on_click([]() {
    std::cout << "Button clicked!\n";
});
```

### Pattern 2: Capture State

```cpp
int threshold = 100;

sensor.on_data([threshold](int value) {
    if (value > threshold) {
        std::cout << "Threshold exceeded!\n";
    }
});
```

### Pattern 3: Modify External State

```cpp
int total = 0;

process_data([&total](int value) {
    total += value;  // Accumulate
});
```

### Pattern 4: STL Algorithms

```cpp
std::vector<int> numbers = {1, 2, 3, 4, 5};

// Filter even numbers
auto is_even = [](int n) { return n % 2 == 0; };
auto it = std::find_if(numbers.begin(), numbers.end(), is_even);

// Transform
std::transform(
    numbers.begin(),
    numbers.end(),
    numbers.begin(),
    [](int n) { return n * 2; }  // Double each element
);

// Sort with custom comparator
std::sort(
    numbers.begin(),
    numbers.end(),
    [](int a, int b) { return a > b; }  // Descending order
);
```

### Pattern 5: Generic Lambda (C++14)

```cpp
// Auto parameters - works with any type!
auto print = [](const auto& x) {
    std::cout << x << std::endl;
};

print(42);          // Prints int
print(3.14);        // Prints double
print("Hello");     // Prints string
```

## Advanced Lambda Features

### Mutable Lambdas

By default, captured-by-value variables are const. Use `mutable` to modify them:

```cpp
int x = 10;

auto lambda = [x]() mutable {
    x++;  // OK with mutable
    std::cout << x << std::endl;
};

lambda();  // Prints 11
lambda();  // Prints 12
std::cout << x << std::endl;  // Still 10 (original unchanged)
```

### Init Captures (C++14)

Initialize variables in capture clause:

```cpp
// Move-capture (useful for unique_ptr)
auto ptr = std::make_unique<int>(42);
auto lambda = [p = std::move(ptr)]() {
    std::cout << *p << std::endl;
};
// ptr is now nullptr, lambda owns the unique_ptr

// Initialize new variable
auto lambda2 = [value = 100]() {
    std::cout << value << std::endl;
};
```

### Recursive Lambdas

```cpp
// Use std::function for recursive lambdas
std::function<int(int)> factorial = [&factorial](int n) -> int {
    return n <= 1 ? 1 : n * factorial(n - 1);
};

int result = factorial(5);  // 120
```

## When to Use Lambdas

### Use Lambdas When:

✓ Callback is simple and used once
✓ You need to capture local variables
✓ Using STL algorithms
✓ ROS2 callbacks (timers, subscriptions, etc.)
✓ Short, inline logic

### Use Regular Functions When:

✓ Function is complex or reused
✓ Function should be testable independently
✓ Function is part of public API
✓ Need to forward-declare

### Example Comparison

```cpp
// Lambda: Good for simple, one-time callbacks
timer_ = create_wall_timer(
    1s,
    [this]() { RCLCPP_INFO(get_logger(), "Tick"); }
);

// Method: Good for complex, reusable logic
timer_ = create_wall_timer(1s, std::bind(&MyNode::complex_callback, this));

void complex_callback() {
    // Many lines of complex logic
    // Can be tested independently
    // Can be called from multiple places
}
```

## Common Pitfalls

### Pitfall 1: Dangling References

```cpp
// BAD: Lambda outlives the captured reference
std::function<void()> create_bad_lambda() {
    int x = 10;
    return [&x]() {  // DANGER: x goes out of scope!
        std::cout << x << std::endl;
    };
}  // x destroyed, lambda has dangling reference!

// GOOD: Capture by value
std::function<void()> create_good_lambda() {
    int x = 10;
    return [x]() {  // Copy x
        std::cout << x << std::endl;
    };
}
```

### Pitfall 2: Capturing `this` in Long-Lived Lambdas

```cpp
class Node {
public:
    void setup() {
        // BAD if callback outlives Node
        some_service->register_callback([this]() {
            this->do_something();  // 'this' might be invalid!
        });
    }

    // BETTER: Use weak_ptr
    void setup_safe() {
        auto weak_this = std::weak_ptr<Node>(shared_from_this());
        some_service->register_callback([weak_this]() {
            if (auto shared_this = weak_this.lock()) {
                shared_this->do_something();  // Safe!
            }
        });
    }
};
```

### Pitfall 3: Unnecessary Copies

```cpp
LargeObject obj;

// BAD: Copies entire object
auto lambda1 = [obj]() {
    // Uses copied obj
};

// GOOD: Capture by reference
auto lambda2 = [&obj]() {
    // Uses original obj
};

// BETTER: Capture specific members
auto lambda3 = [value = obj.getValue()]() {
    // Only copies what's needed
};
```

### Pitfall 4: Modifying Captured Values

```cpp
int x = 10;

// ERROR: Can't modify captured-by-value without 'mutable'
auto lambda = [x]() {
    x++;  // ERROR!
};

// Fix 1: Use mutable
auto lambda1 = [x]() mutable {
    x++;  // OK, but doesn't affect original x
};

// Fix 2: Capture by reference
auto lambda2 = [&x]() {
    x++;  // OK, modifies original x
};
```

## Best Practices

### DO ✓

```cpp
// Capture by reference for large objects (if lambda doesn't outlive them)
LargeObject obj;
auto lambda = [&obj]() { obj.process(); };

// Use [this] in member functions
auto lambda = [this]() { this->member_function(); };

// Prefer specific captures over [=] or [&]
int x, y;
auto lambda = [x, &y]() { /* use x and y */ };

// Use auto for parameter types (C++14)
auto lambda = [](const auto& value) { /* works with any type */ };
```

### DON'T ✗

```cpp
// Don't capture by reference if lambda outlives scope
std::function<void()> bad() {
    int x = 10;
    return [&x]() { };  // DANGLING REFERENCE!
}

// Don't capture [=] or [&] unless you really need everything
auto bad_lambda = [=]() { use_one_var(); };  // Captures everything!

// Don't forget 'mutable' if you need to modify captured values
int x = 0;
auto bad = [x]() { x++; };  // ERROR!
```

## Lambda Function Examples

### Example 1: ROS2 Multi-Callback Node

```cpp
class MultiCallbackNode : public rclcpp::Node {
public:
    MultiCallbackNode() : Node("multi_callback"), count_(0) {
        // Timer with lambda
        timer_ = create_wall_timer(
            1s,
            [this]() {
                count_++;
                RCLCPP_INFO(get_logger(), "Count: %d", count_);
            }
        );

        // Subscription with lambda
        sub_ = create_subscription<std_msgs::msg::Int32>(
            "input",
            10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                RCLCPP_INFO(get_logger(), "Received: %d", msg->data);
                process_data(msg->data);
            }
        );

        // Publisher (no lambda needed, but showing pattern)
        pub_ = create_publisher<std_msgs::msg::Int32>("output", 10);
    }

private:
    void process_data(int value) {
        auto msg = std::make_unique<std_msgs::msg::Int32>();
        msg->data = value * 2;
        pub_->publish(std::move(msg));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    int count_;
};
```

### Example 2: Data Processing with Lambdas

```cpp
std::vector<SensorReading> filter_and_process(
    const std::vector<SensorReading>& readings,
    double threshold
) {
    std::vector<SensorReading> filtered;

    // Filter using lambda
    std::copy_if(
        readings.begin(),
        readings.end(),
        std::back_inserter(filtered),
        [threshold](const SensorReading& r) {
            return r.value > threshold && r.valid;
        }
    );

    // Transform using lambda
    std::transform(
        filtered.begin(),
        filtered.end(),
        filtered.begin(),
        [](SensorReading r) {
            r.value = normalize(r.value);
            return r;
        }
    );

    return filtered;
}
```

## Summary

**Key Takeaways:**
- Lambdas are anonymous inline functions
- Capture allows access to enclosing scope variables
- `[=]` captures by value, `[&]` by reference
- ROS2 uses lambdas for all callbacks
- Use `[this]` to capture class instance in member functions
- Watch out for dangling references with captured variables
- Lambdas make code more concise and readable

**Critical for ROS2:**
- Timer callbacks: `[this]() { ... }`
- Subscriptions: `[this](const MsgType::SharedPtr msg) { ... }`
- Services: `[this](Request, Response) { ... }`

## Practice Exercises

1. Write a ROS2 node with timer and subscription using lambdas
2. Use lambdas with STL algorithms to process sensor data
3. Compare lambda vs member function callbacks

## What's Next?

- **Next Lesson**: [Move Semantics](07-move-semantics.md)
- **Related**: [Smart Pointers](04-smart-pointers.md) - Often used together
- **Practice**: [Exercise 3: Lambdas](../exercises/ex03-lambdas.md)

---

**Master lambdas and ROS2 callbacks become second nature!**
