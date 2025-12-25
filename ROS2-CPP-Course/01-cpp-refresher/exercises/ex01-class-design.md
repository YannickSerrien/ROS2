# Exercise 1: Class Design - Sensor Hierarchy

## Difficulty: Beginner

## Learning Objectives
- Practice OOP principles
- Implement class inheritance
- Use virtual functions
- Apply const correctness

## Problem Statement

Design and implement a sensor class hierarchy for a robot system. The system needs to support different types of sensors (Temperature, Distance, and Camera) that all share common functionality but have specialized behavior.

## Requirements

### Base Class: `Sensor`
Create an abstract base class with:
- Protected member: `std::string name_`
- Protected member: `bool is_active_`
- Constructor taking sensor name
- Virtual destructor
- Pure virtual method: `double read()` - Returns sensor reading
- Virtual method: `std::string getType()` - Returns sensor type
- Method: `std::string getName()` - Returns sensor name (const)
- Method: `bool isActive()` - Returns activation status (const)
- Method: `void setActive(bool active)` - Sets activation status

### Derived Class 1: `TemperatureSensor`
- Simulates temperature readings (random 20-30°C)
- Override `read()` to return temperature
- Override `getType()` to return "Temperature"

### Derived Class 2: `DistanceSensor`
- Simulates distance readings (random 0-100 cm)
- Override `read()` to return distance
- Override `getType()` to return "Distance"

### Derived Class 3: `CameraSensor`
- Returns 1.0 if "image captured", 0.0 otherwise
- Override `read()` to return capture status
- Override `getType()` to return "Camera"

### Main Program
Create a `main()` function that:
1. Creates a vector of `unique_ptr<Sensor>`
2. Adds one of each sensor type
3. Loops through all sensors and:
   - Prints sensor name and type
   - Reads and prints the value
   - Uses polymorphism to call the correct `read()` method

## Starter Code

```cpp
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <cstdlib>
#include <ctime>

// TODO: Implement Sensor base class

// TODO: Implement TemperatureSensor

// TODO: Implement DistanceSensor

// TODO: Implement CameraSensor

int main() {
    // Initialize random seed
    std::srand(std::time(nullptr));

    // TODO: Create vector of sensors

    // TODO: Add sensors to vector

    // TODO: Loop through sensors and print info

    return 0;
}
```

## Expected Output

```
Sensor: temp1
  Type: Temperature
  Reading: 25.34
  Status: Active

Sensor: dist1
  Type: Distance
  Reading: 45.67
  Status: Active

Sensor: cam1
  Type: Camera
  Reading: 1.00
  Status: Active
```

## Hints

1. Use `virtual` keyword for base class methods
2. Use `override` keyword in derived classes
3. Remember to make the base class destructor virtual
4. Use `const` for methods that don't modify the object
5. Use `std::rand()` for random numbers
6. Formula: `rand() % (max - min + 1) + min` for random in range

## Extension Challenges

Once you complete the basic exercise:

1. **Add calibration**: Add a `calibrate()` method that each sensor implements differently
2. **Add timestamp**: Add a timestamp to each reading
3. **Add units**: Make `read()` return a struct with value and units
4. **Add error handling**: Add a method to simulate sensor failures

## Compilation

```bash
g++ -std=c++17 -Wall ex01-solution.cpp -o ex01
./ex01
```

## Solution

See `solutions/ex01-solution.cpp` for the complete implementation.
