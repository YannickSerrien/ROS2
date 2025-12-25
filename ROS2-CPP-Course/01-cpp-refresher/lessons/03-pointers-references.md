# Lesson 3: Pointers and References

## Learning Objectives

- Understand pointers and references in C++
- Know when to use pointers vs references
- Avoid common pointer pitfalls
- Use pointers and references in ROS2

## Pointers vs References Quick Comparison

| Feature | Pointer | Reference |
|---------|---------|-----------|
| Syntax | `int* ptr` | `int& ref` |
| Can be null | Yes (`nullptr`) | No |
| Can be reassigned | Yes | No (always refers to same object) |
| Requires dereferencing | Yes (`*ptr`) | No (automatic) |
| Memory address | `&variable` | Same as variable |

## References

### Basic Usage

```cpp
int x = 10;
int& ref = x;  // ref is an alias for x

ref = 20;  // Modifies x
std::cout << x << std::endl;  // Prints: 20
```

### Function Parameters (Most Common Use)

```cpp
// Pass by value (copies parameter)
void badModify(int value) {
    value = 100;  // Doesn't affect original
}

// Pass by reference (modifies original)
void goodModify(int& value) {
    value = 100;  // Modifies original!
}

// Pass by const reference (efficient, read-only)
void process(const std::string& data) {
    // Can read data, cannot modify
    std::cout << data << std::endl;
}

int x = 5;
badModify(x);   // x still 5
goodModify(x);  // x now 100

std::string msg = "Large message";
process(msg);  // No copy, efficient!
```

### ROS2 Usage

```cpp
// ROS2 callback with const reference (standard pattern)
void callback(const std_msgs::msg::String::SharedPtr msg) {
    // msg is efficient (no copy)
    RCLCPP_INFO(logger_, "Received: %s", msg->data.c_str());
}
```

## Pointers

### Basic Usage

```cpp
int x = 42;
int* ptr = &x;  // ptr holds address of x

std::cout << ptr << std::endl;   // Prints address
std::cout << *ptr << std::endl;  // Prints 42 (dereference)

*ptr = 100;  // Modify x through pointer
std::cout << x << std::endl;  // Prints: 100
```

### Nullptr

```cpp
int* ptr = nullptr;  // Null pointer

if (ptr != nullptr) {
    *ptr = 10;  // Safe: only dereference if not null
}

// Always check before dereferencing!
```

### Pointers vs Smart Pointers

```cpp
// Raw pointer (avoid for ownership!)
int* raw = new int(42);
delete raw;  // Manual cleanup - error-prone!

// Smart pointer (preferred!)
auto smart = std::make_unique<int>(42);
// Automatic cleanup!
```

## When to Use Each

### Use References When:
✓ Function parameters (read or modify)
✓ Returning from functions (if lifetime is guaranteed)
✓ Alias for readability
✓ Never null

### Use Pointers When:
✓ Optional value (can be nullptr)
✓ Need to reassign
✓ Interfacing with C libraries
✓ Ownership semantics (use smart pointers!)

### Examples

```cpp
// Reference: Always valid, cannot be null
void processData(const Data& data) {
    // data always valid, no null check needed
}

// Pointer: Optional parameter
void processOptionalData(const Data* data) {
    if (data != nullptr) {
        // Process data
    }
}

// Smart pointer: Ownership transfer
std::unique_ptr<Data> createData() {
    return std::make_unique<Data>();
}
```

## Common Pitfalls

### Dangling Reference

```cpp
// BAD: Returns reference to local variable
const std::string& getDangling() {
    std::string local = "bad";
    return local;  // DANGER: local destroyed!
}

// GOOD: Return by value (move semantics make it efficient)
std::string getGood() {
    return "good";
}
```

### Null Pointer Dereference

```cpp
int* ptr = nullptr;
*ptr = 10;  // CRASH!

// Always check:
if (ptr != nullptr) {
    *ptr = 10;  // Safe
}
```

### Pointer Arithmetic (Usually Avoid)

```cpp
int arr[] = {1, 2, 3, 4, 5};
int* ptr = arr;

ptr++;  // Points to arr[1]
ptr += 2;  // Points to arr[3]

// Prefer iterators or range-based for loops!
for (int value : arr) {
    std::cout << value << std::endl;
}
```

## ROS2 Patterns

```cpp
class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        subscription_ = create_subscription<std_msgs::msg::String>(
            "topic",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg) {  // Smart pointer!
                // msg is a shared_ptr, automatically managed
                processMessage(*msg);  // Dereference to get reference
            }
        );
    }

private:
    void processMessage(const std_msgs::msg::String& msg) {  // Reference parameter
        RCLCPP_INFO(get_logger(), "Data: %s", msg.data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```

## Summary

**Key Takeaways:**
- References are aliases, pointers hold addresses
- Prefer references for function parameters
- Use smart pointers over raw pointers
- Check pointers for nullptr before dereferencing
- ROS2 uses both: smart pointers for ownership, references for efficiency

## What's Next?

- **Next Lesson**: [Smart Pointers](04-smart-pointers.md)
- **Practice**: Exercises on pointers and references
