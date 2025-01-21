#include <vector>

template <typename T>
class FixedSizeQueue {
 private:
  std::vector<T> queue;
  size_t size_;

 public:
  void fill(size_t size, const T &initial_value) {
    size_ = size;
    queue.resize(size, initial_value);
  }

  // Enqueue a new element, remove the oldest element, and return it
  T enqueue(const T &new_value) {
    if (size_ == 0) {
      return new_value;
    }
    T oldest_element = queue.front();  // The oldest element to be returned
    queue.erase(queue.begin());        // Remove the oldest element
    queue.push_back(new_value);        // Add the new element to the end
    return oldest_element;
  }
};
