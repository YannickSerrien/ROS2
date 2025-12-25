# Lesson 8: STL Essentials

## Learning Objectives

- Master essential STL containers
- Use STL algorithms effectively
- Understand iterators
- Apply STL in ROS2 code

## Standard Template Library (STL)

The STL provides containers, algorithms, and iterators - essential for modern C++.

## Essential Containers

### std::vector - Dynamic Array

```cpp
#include <vector>

// Create vector
std::vector<int> numbers = {1, 2, 3, 4, 5};

// Add elements
numbers.push_back(6);        // Add to end
numbers.insert(numbers.begin(), 0);  // Insert at beginning

// Access
int first = numbers[0];      // No bounds checking
int second = numbers.at(1);  // With bounds checking

// Size
size_t count = numbers.size();
bool empty = numbers.empty();

// Iterate
for (const auto& num : numbers) {
    std::cout << num << " ";
}

// Remove
numbers.pop_back();          // Remove last
numbers.erase(numbers.begin());  // Remove first
numbers.clear();             // Remove all
```

### std::map - Key-Value Pairs

```cpp
#include <map>

// Create map
std::map<std::string, int> ages;

// Insert
ages["Alice"] = 30;
ages["Bob"] = 25;
ages.insert({"Charlie", 35});

// Access
int alice_age = ages["Alice"];

// Check if key exists
if (ages.find("David") != ages.end()) {
    // Key exists
}

// Iterate
for (const auto& [name, age] : ages) {  // C++17 structured binding
    std::cout << name << ": " << age << "\n";
}

// Remove
ages.erase("Bob");
```

### std::unordered_map - Hash Map (Faster)

```cpp
#include <unordered_map>

// Same interface as map, but O(1) average lookup
std::unordered_map<std::string, double> scores;
scores["test1"] = 95.5;
scores["test2"] = 87.3;
```

### std::set - Unique Elements

```cpp
#include <set>

std::set<int> unique_numbers = {1, 2, 2, 3, 3, 3};  // {1, 2, 3}

unique_numbers.insert(4);
unique_numbers.erase(2);

// Check membership
if (unique_numbers.count(3) > 0) {
    // 3 is in set
}
```

### std::queue - FIFO

```cpp
#include <queue>

std::queue<std::string> messages;
messages.push("msg1");
messages.push("msg2");

std::string first = messages.front();
messages.pop();  // Remove front
```

### std::deque - Double-ended Queue

```cpp
#include <deque>

std::deque<int> dq = {3, 4};
dq.push_front(2);  // Add to front: {2, 3, 4}
dq.push_back(5);   // Add to back: {2, 3, 4, 5}
```

## STL Algorithms

### Common Algorithms

```cpp
#include <algorithm>
#include <vector>

std::vector<int> vec = {5, 2, 8, 1, 9};

// Sort
std::sort(vec.begin(), vec.end());  // {1, 2, 5, 8, 9}

// Reverse
std::reverse(vec.begin(), vec.end());  // {9, 8, 5, 2, 1}

// Find
auto it = std::find(vec.begin(), vec.end(), 5);
if (it != vec.end()) {
    std::cout << "Found: " << *it << "\n";
}

// Count
int count = std::count(vec.begin(), vec.end(), 2);

// Min/Max
int min_val = *std::min_element(vec.begin(), vec.end());
int max_val = *std::max_element(vec.begin(), vec.end());

// Remove
vec.erase(std::remove(vec.begin(), vec.end(), 5), vec.end());

// Transform
std::vector<int> doubled(vec.size());
std::transform(vec.begin(), vec.end(), doubled.begin(),
    [](int x) { return x * 2; });

// Filter with copy_if
std::vector<int> evens;
std::copy_if(vec.begin(), vec.end(), std::back_inserter(evens),
    [](int x) { return x % 2 == 0; });
```

### Sorting with Custom Comparator

```cpp
struct Person {
    std::string name;
    int age;
};

std::vector<Person> people = {
    {"Alice", 30},
    {"Bob", 25},
    {"Charlie", 35}
};

// Sort by age
std::sort(people.begin(), people.end(),
    [](const Person& a, const Person& b) {
        return a.age < b.age;
    });
```

## Iterators

```cpp
std::vector<int> vec = {1, 2, 3, 4, 5};

// Iterator
std::vector<int>::iterator it = vec.begin();
while (it != vec.end()) {
    std::cout << *it << " ";
    ++it;
}

// Auto (easier)
for (auto it = vec.begin(); it != vec.end(); ++it) {
    *it *= 2;  // Modify elements
}

// Range-based for (easiest)
for (auto& value : vec) {
    value *= 2;
}
```

## ROS2 Usage Examples

### Storing Sensor Data

```cpp
class SensorNode : public rclcpp::Node {
public:
    SensorNode() : Node("sensor_node") {
        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                // Store recent scans (keep last 10)
                recent_scans_.push_back(*msg);
                if (recent_scans_.size() > 10) {
                    recent_scans_.erase(recent_scans_.begin());
                }

                // Find minimum distance
                auto min_it = std::min_element(
                    msg->ranges.begin(),
                    msg->ranges.end()
                );
                RCLCPP_INFO(get_logger(), "Min distance: %.2f", *min_it);
            }
        );
    }

private:
    std::vector<sensor_msgs::msg::LaserScan> recent_scans_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};
```

### Managing Node Parameters

```cpp
// Map for configuration
std::map<std::string, double> config = {
    {"max_speed", 1.0},
    {"acceleration", 0.5},
    {"timeout", 5.0}
};

// Update from parameters
for (const auto& [key, value] : config) {
    declare_parameter(key, value);
}
```

### Queue for Message Processing

```cpp
class MessageProcessor : public rclcpp::Node {
public:
    MessageProcessor() : Node("processor") {
        subscription_ = create_subscription<std_msgs::msg::String>(
            "input",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                message_queue_.push(msg->data);
                process_queue();
            }
        );
    }

private:
    void process_queue() {
        while (!message_queue_.empty()) {
            std::string msg = message_queue_.front();
            message_queue_.pop();
            // Process message
            RCLCPP_INFO(get_logger(), "Processing: %s", msg.c_str());
        }
    }

    std::queue<std::string> message_queue_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```

## Best Practices

### Choose Right Container

| Need | Container |
|------|-----------|
| Dynamic array | `std::vector` |
| Key-value lookup | `std::map` or `std::unordered_map` |
| Unique elements | `std::set` |
| FIFO queue | `std::queue` |
| Stack (LIFO) | `std::stack` |
| Both ends | `std::deque` |

### Reserve Space for Vectors

```cpp
std::vector<int> vec;
vec.reserve(1000);  // Avoid reallocations
for (int i = 0; i < 1000; ++i) {
    vec.push_back(i);
}
```

### Use Const Correctness

```cpp
// Const iterator (can't modify)
for (std::vector<int>::const_iterator it = vec.cbegin();
     it != vec.cend(); ++it) {
    // *it = 10;  // ERROR
}

// Const reference in range-for
for (const auto& item : vec) {
    // item = 10;  // ERROR
}
```

## Summary

**Key Takeaways:**
- `vector` is the default container (dynamic array)
- `map`/`unordered_map` for key-value storage
- `set` for unique elements
- STL algorithms avoid manual loops
- Iterators provide generic access
- ROS2 uses STL containers extensively

**Critical for ROS2:**
- Store sensor data in vectors
- Use maps for configuration
- Algorithms for data processing
- Efficient with proper container choice

## What's Next?

- **Next Lesson**: [Namespaces and Modules](09-namespaces-modules.md)
- **Practice**: Use STL in ROS2 nodes
