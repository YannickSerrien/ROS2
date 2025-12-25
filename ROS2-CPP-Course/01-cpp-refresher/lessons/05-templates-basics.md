# Lesson 5: Templates Basics

## Learning Objectives

- Understand C++ templates
- Write function templates
- Create class templates
- Use templates in ROS2 contexts

## What Are Templates?

Templates enable **generic programming** - writing code that works with any type.

### Python Comparison

```python
# Python: Works with any type automatically
def get_max(a, b):
    return a if a > b else b

print(get_max(5, 3))      # Works with int
print(get_max(5.5, 3.3))  # Works with float
```

**C++: Needs templates for generic code**

## Function Templates

###Basic Function Template

```cpp
// Template definition
template<typename T>
T get_max(T a, T b) {
    return (a > b) ? a : b;
}

// Usage
int i = get_max(10, 20);           // T = int
double d = get_max(3.14, 2.71);    // T = double
std::string s = get_max(std::string("a"), std::string("b"));  // T = std::string
```

### Multiple Template Parameters

```cpp
template<typename T, typename U>
auto multiply(T a, U b) {
    return a * b;
}

auto result = multiply(3, 4.5);  // T=int, U=double, result=double
```

### ROS2 Example

```cpp
// Generic publisher creation
template<typename MessageT>
auto create_simple_publisher(rclcpp::Node* node, const std::string& topic) {
    return node->create_publisher<MessageT>(topic, 10);
}

// Usage
auto pub = create_simple_publisher<std_msgs::msg::String>(node, "topic");
```

## Class Templates

### Basic Class Template

```cpp
template<typename T>
class Container {
public:
    void add(const T& item) {
        items_.push_back(item);
    }

    T get(size_t index) const {
        return items_[index];
    }

    size_t size() const {
        return items_.size();
    }

private:
    std::vector<T> items_;
};

// Usage
Container<int> int_container;
int_container.add(10);
int_container.add(20);

Container<std::string> string_container;
string_container.add("hello");
string_container.add("world");
```

### Template with Default Parameter

```cpp
template<typename T, typename Container = std::vector<T>>
class Stack {
public:
    void push(const T& value) {
        container_.push_back(value);
    }

    T pop() {
        T value = container_.back();
        container_.pop_back();
        return value;
    }

private:
    Container container_;
};

Stack<int> stack;  // Uses std::vector<int> by default
```

## STL Containers (Built-in Templates)

```cpp
// Vector: dynamic array
std::vector<int> numbers = {1, 2, 3, 4, 5};
numbers.push_back(6);

// Map: key-value pairs
std::map<std::string, int> ages;
ages["Alice"] = 30;
ages["Bob"] = 25;

// Set: unique elements
std::set<int> unique_numbers = {1, 2, 2, 3, 3, 3};  // {1, 2, 3}

// Queue
std::queue<std::string> message_queue;
message_queue.push("msg1");
message_queue.push("msg2");
```

## ROS2 Templates

ROS2 APIs are heavily templated:

```cpp
// Publisher template
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

// Subscription template
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

// Service client template
rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;

// Generic node creation
template<typename NodeT>
void run_node() {
    auto node = std::make_shared<NodeT>();
    rclcpp::spin(node);
}

run_node<MyRobotNode>();
```

## Template Specialization

```cpp
// Generic template
template<typename T>
class Sensor {
public:
    T read() {
        // Generic reading logic
        return T();
    }
};

// Specialized for specific type
template<>
class Sensor<double> {
public:
    double read() {
        // Special logic for double
        return 23.5;  // Temperature reading
    }
};

Sensor<int> s1;      // Uses generic
Sensor<double> s2;   // Uses specialization
```

## Best Practices

### DO ✓
```cpp
// Use typename for clarity
template<typename T>  // Preferred
// vs
template<class T>     // Also valid, but typename is clearer

// Use concepts (C++20) for constraints
template<typename T>
requires std::integral<T>  // T must be integer type
T add(T a, T b) {
    return a + b;
}
```

### Common Pitfalls

```cpp
// Templates must be in header files (usually)
// Implementation needs to be visible to compiler

// template_class.hpp
template<typename T>
class MyClass {
public:
    void doSomething(T value);  // Declaration
};

// Implementation in same header (or .tpp file included at end)
template<typename T>
void MyClass<T>::doSomething(T value) {
    // Implementation
}
```

## Summary

**Key Takeaways:**
- Templates enable generic, reusable code
- Function templates: `template<typename T> T func(T a)`
- Class templates: `template<typename T> class MyClass`
- ROS2 uses templates extensively
- STL containers are class templates

**Critical for ROS2:**
- Publishers/subscribers are templates
- Message types are template parameters
- Understanding templates is essential for using ROS2 APIs

## What's Next?

- **Next Lesson**: [Lambda Functions](06-lambda-functions.md)
- **Practice**: Create templated data structures
