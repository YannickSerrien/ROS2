# Lesson 4: Smart Pointers

## Learning Objectives

- Understand the problems with raw pointers
- Master `std::unique_ptr` for exclusive ownership
- Master `std::shared_ptr` for shared ownership
- Know when to use `std::weak_ptr`
- Apply RAII pattern for resource management
- Use smart pointers correctly in ROS2

## Introduction

**Smart pointers** are the most important feature of modern C++ for ROS2 development. They solve memory management problems by automatically cleaning up resources. ROS2 uses smart pointers extensively - understanding them is non-negotiable!

## The Problem with Raw Pointers

### Manual Memory Management (Dangerous!)

```cpp
void problematic_function() {
    MyClass* ptr = new MyClass();  // Allocate memory

    // ... do work ...

    if (error_condition) {
        return;  // MEMORY LEAK! Forgot to delete
    }

    // ... more work ...

    delete ptr;  // Manual cleanup - easy to forget!
}
```

**Problems:**
- ❌ Memory leaks if you forget `delete`
- ❌ Double-delete crashes
- ❌ Dangling pointers (using after delete)
- ❌ Exception safety issues

### Python Comparison

```python
# Python - automatic garbage collection
def python_function():
    obj = MyClass()  # Created
    # ...
    # Automatically cleaned up!
```

Modern C++ can do this too - with smart pointers!

## RAII: Resource Acquisition Is Initialization

**Core Principle**: Resources are tied to object lifetime.

- **Acquisition**: Get resource in constructor
- **Release**: Release resource in destructor
- **Automatic**: Destructor called when scope ends

```cpp
{
    std::unique_ptr<MyClass> ptr = std::make_unique<MyClass>();
    // ptr created, memory allocated

    // ... use ptr ...

}  // Scope ends, destructor runs, memory freed automatically!
```

No manual cleanup needed!

## std::unique_ptr - Exclusive Ownership

`unique_ptr` represents **exclusive ownership** of a resource.

### Basic Usage

```cpp
#include <memory>

// Create unique_ptr
std::unique_ptr<int> ptr = std::make_unique<int>(42);

// Access value
std::cout << *ptr << std::endl;  // 42

// Reset (deletes old, assigns new)
ptr.reset(new int(100));

// Release ownership (returns raw pointer, doesn't delete)
int* raw = ptr.release();  // ptr is now nullptr

// Delete manually when unique_ptr doesn't own it
delete raw;
```

### Key Characteristics

- **Cannot be copied** (exclusive ownership)
- **Can be moved** (transfer ownership)
- **Zero overhead** (same as raw pointer)
- **Automatically deletes** when destroyed

### Example: Unique Ownership

```cpp
class Sensor {
public:
    Sensor(const std::string& name) : name_(name) {
        std::cout << "Sensor " << name_ << " created\n";
    }

    ~Sensor() {
        std::cout << "Sensor " << name_ << " destroyed\n";
    }

    void read() {
        std::cout << "Reading from " << name_ << "\n";
    }

private:
    std::string name_;
};

void demo_unique_ptr() {
    auto sensor = std::make_unique<Sensor>("GPS");
    sensor->read();

    // sensor automatically deleted here!
}  // Destructor called, prints "Sensor GPS destroyed"
```

### Moving unique_ptr

```cpp
std::unique_ptr<Sensor> sensor1 = std::make_unique<Sensor>("IMU");

// Transfer ownership with std::move
std::unique_ptr<Sensor> sensor2 = std::move(sensor1);

// sensor1 is now nullptr
// sensor2 owns the Sensor
```

### unique_ptr in Functions

```cpp
// Return unique_ptr (transfer ownership out)
std::unique_ptr<Sensor> create_sensor() {
    return std::make_unique<Sensor>("Lidar");
}

// Accept unique_ptr (take ownership)
void consume_sensor(std::unique_ptr<Sensor> sensor) {
    sensor->read();
    // sensor deleted when function ends
}

// Accept raw pointer/reference (borrow, don't own)
void use_sensor(Sensor* sensor) {
    sensor->read();
    // Caller still owns sensor
}

// Usage
auto s = create_sensor();
use_sensor(s.get());      // Borrow (get raw pointer)
consume_sensor(std::move(s));  // Give away ownership
// s is now nullptr
```

## std::shared_ptr - Shared Ownership

`shared_ptr` allows **multiple owners** of the same resource.

### Reference Counting

```cpp
std::shared_ptr<int> ptr1 = std::make_shared<int>(42);
std::cout << ptr1.use_count() << std::endl;  // 1

{
    std::shared_ptr<int> ptr2 = ptr1;  // Share ownership
    std::cout << ptr1.use_count() << std::endl;  // 2
}  // ptr2 destroyed, count decreases

std::cout << ptr1.use_count() << std::endl;  // 1
// Resource deleted when last shared_ptr is destroyed
```

### Basic Usage

```cpp
// Create shared_ptr (prefer make_shared)
auto ptr = std::make_shared<MyClass>();

// Copy creates new reference
auto ptr2 = ptr;  // Both point to same object

// Check reference count
std::cout << ptr.use_count() << std::endl;

// Reset (decrements count, may delete)
ptr.reset();

// Get raw pointer
MyClass* raw = ptr.get();
```

### Example: Shared Ownership

```cpp
class Node {
public:
    Node(int value) : value_(value) {
        std::cout << "Node " << value_ << " created\n";
    }

    ~Node() {
        std::cout << "Node " << value_ << " destroyed\n";
    }

    int value_;
};

void demo_shared_ptr() {
    auto node1 = std::make_shared<Node>(1);
    std::cout << "Count: " << node1.use_count() << "\n";  // 1

    {
        auto node2 = node1;  // Share
        std::cout << "Count: " << node1.use_count() << "\n";  // 2

        auto node3 = node1;  // Share again
        std::cout << "Count: " << node1.use_count() << "\n";  // 3
    }  // node2 and node3 destroyed

    std::cout << "Count: " << node1.use_count() << "\n";  // 1
}  // node1 destroyed, Node deleted
```

### shared_ptr in ROS2

ROS2 nodes, publishers, and subscribers use `shared_ptr`:

```cpp
// ROS2 node creation
auto node = std::make_shared<rclcpp::Node>("my_node");

// Publisher (shared ownership)
auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
// Type: rclcpp::Publisher<...>::SharedPtr

// Multiple references OK
auto pub_copy = publisher;  // Both point to same publisher
```

## std::weak_ptr - Non-Owning Reference

`weak_ptr` observes but doesn't own. It breaks circular references.

### The Circular Reference Problem

```cpp
class B;  // Forward declaration

class A {
public:
    std::shared_ptr<B> b_ptr;
    ~A() { std::cout << "A destroyed\n"; }
};

class B {
public:
    std::shared_ptr<A> a_ptr;  // PROBLEM: Circular reference!
    ~B() { std::cout << "B destroyed\n"; }
};

void circular_problem() {
    auto a = std::make_shared<A>();
    auto b = std::make_shared<B>();

    a->b_ptr = b;
    b->a_ptr = a;  // Circular reference!

    // Neither destructor called - MEMORY LEAK!
}  // a and b both have use_count > 0
```

### Solution with weak_ptr

```cpp
class B;

class A {
public:
    std::shared_ptr<B> b_ptr;
    ~A() { std::cout << "A destroyed\n"; }
};

class B {
public:
    std::weak_ptr<A> a_ptr;  // Weak reference - doesn't increase count
    ~B() { std::cout << "B destroyed\n"; }
};

void weak_solution() {
    auto a = std::make_shared<A>();
    auto b = std::make_shared<B>();

    a->b_ptr = b;
    b->a_ptr = a;  // Weak reference - OK!

    // Both destructors called - NO LEAK!
}
```

### Using weak_ptr

```cpp
std::shared_ptr<int> shared = std::make_shared<int>(42);
std::weak_ptr<int> weak = shared;  // Observe, don't own

// To use weak_ptr, convert to shared_ptr
if (auto locked = weak.lock()) {  // Returns shared_ptr if alive
    std::cout << *locked << std::endl;  // Safe to use
} else {
    std::cout << "Object was deleted\n";
}

// Check if still alive
if (weak.expired()) {
    std::cout << "Object gone\n";
}
```

## Choosing the Right Smart Pointer

| Use Case | Smart Pointer | Reason |
|----------|---------------|--------|
| Single owner | `unique_ptr` | Most efficient, clear ownership |
| Shared ownership | `shared_ptr` | Multiple owners need access |
| Cache/observer | `weak_ptr` | Observe without ownership |
| ROS2 nodes/publishers | `shared_ptr` | Required by ROS2 API |
| Local variables | `unique_ptr` or stack | Simplest |
| Return from function | `unique_ptr` | Transfer ownership clearly |

### Decision Flow

```
Need a pointer?
    ├─ Single owner? → unique_ptr
    ├─ Multiple owners? → shared_ptr
    ├─ Just observing? → weak_ptr
    └─ Don't need pointer? → Use stack object (MyClass obj;)
```

## Smart Pointers in ROS2

### ROS2 Node Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SmartNode : public rclcpp::Node {
public:
    SmartNode() : Node("smart_node") {
        // Publisher: shared_ptr (ROS2 API requirement)
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

        // Timer: shared_ptr
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { this->publish_message(); }
        );
    }

private:
    void publish_message() {
        // Message: unique_ptr for efficiency
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello!";

        // Move ownership to publisher (zero-copy)
        publisher_->publish(std::move(msg));
    }

    // Members: shared_ptr (ROS2 types)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create node: shared_ptr
    auto node = std::make_shared<SmartNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Message Publishing Patterns

```cpp
// Pattern 1: Create and move (most efficient)
auto msg = std::make_unique<MsgType>();
msg->data = "value";
publisher->publish(std::move(msg));

// Pattern 2: Stack allocation and copy (simple, but copies)
MsgType msg;
msg.data = "value";
publisher->publish(msg);

// Pattern 3: Shared ptr (when multiple owners need access)
auto msg = std::make_shared<MsgType>();
msg->data = "value";
publisher->publish(msg);  // Copies shared_ptr, not data
```

## Common Patterns

### Factory Pattern

```cpp
// Return unique_ptr from factory
std::unique_ptr<Sensor> create_sensor(const std::string& type) {
    if (type == "GPS") {
        return std::make_unique<GPSSensor>();
    } else if (type == "IMU") {
        return std::make_unique<IMUSensor>();
    }
    return nullptr;
}

// Usage
auto sensor = create_sensor("GPS");
if (sensor) {
    sensor->read();
}
```

### Polymorphism with Smart Pointers

```cpp
class Shape {
public:
    virtual ~Shape() = default;
    virtual void draw() const = 0;
};

class Circle : public Shape {
public:
    void draw() const override {
        std::cout << "Drawing circle\n";
    }
};

class Square : public Shape {
public:
    void draw() const override {
        std::cout << "Drawing square\n";
    }
};

// Store in vector
std::vector<std::unique_ptr<Shape>> shapes;
shapes.push_back(std::make_unique<Circle>());
shapes.push_back(std::make_unique<Square>());

for (const auto& shape : shapes) {
    shape->draw();  // Polymorphic call
}
```

## Best Practices

### DO ✓

```cpp
// Use make_unique/make_shared
auto ptr = std::make_unique<MyClass>();

// Use unique_ptr by default
std::unique_ptr<Data> data = load_data();

// Move when transferring ownership
consume(std::move(data));

// Return unique_ptr from factories
std::unique_ptr<Widget> create_widget();

// Use weak_ptr to break cycles
std::weak_ptr<Parent> parent_;
```

### DON'T ✗

```cpp
// Don't use new directly
std::unique_ptr<MyClass> ptr(new MyClass());  // Prefer make_unique

// Don't use raw pointers for ownership
MyClass* ptr = new MyClass();  // Use smart pointers!

// Don't copy unique_ptr (you can't anyway)
auto ptr2 = ptr1;  // ERROR: deleted copy constructor

// Don't use shared_ptr unless you need shared ownership
std::shared_ptr<int> x = std::make_shared<int>(5);  // Overkill for int!

// Don't dereference weak_ptr directly
std::weak_ptr<int> weak = ...;
*weak;  // ERROR: Can't dereference weak_ptr
```

## Performance Considerations

| Smart Pointer | Overhead | Use When |
|---------------|----------|----------|
| `unique_ptr` | None | Default choice |
| `shared_ptr` | Reference counting | Actually need shared ownership |
| `weak_ptr` | None (observes shared_ptr) | Breaking cycles, caching |
| Raw pointer | None | Borrowing (non-owning) |

## Summary

**Key Takeaways:**
- Smart pointers automate memory management via RAII
- `unique_ptr`: Exclusive ownership, zero overhead
- `shared_ptr`: Shared ownership, reference counting
- `weak_ptr`: Non-owning observer, breaks cycles
- ROS2 uses `shared_ptr` for nodes, publishers, etc.
- Use `unique_ptr` for messages and local ownership
- Prefer `make_unique`/`make_shared` over `new`

**Critical for ROS2:**
- Nodes: `std::make_shared<rclcpp::Node>()`
- Publishers/Subscribers: `shared_ptr`
- Messages: `unique_ptr` + `std::move()`

## Practice Exercises

Try these to solidify your understanding:

1. Convert code using raw pointers to smart pointers
2. Implement a resource manager using `unique_ptr`
3. Create a subscriber in ROS2 and understand the `shared_ptr` usage

## What's Next?

- **Next Lesson**: [Templates Basics](05-templates-basics.md)
- **Related**: [Lambda Functions](06-lambda-functions.md) - Often used with smart pointers
- **Practice**: [Exercise 2: Smart Pointers](../exercises/ex02-smart-pointers.md)

---

**Master smart pointers and you've mastered modern C++ memory management!**
