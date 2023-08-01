
#ifndef FILTER_H
#define FILTER_H

#include <cassert>
#include <iostream>
#include <utility>
#include <deque>

class MovingWindowFilter {
 public:

  MovingWindowFilter() {}

  MovingWindowFilter(int window_size) : window_size_(window_size) {
    assert(window_size_ > 0);
    sum_ = 0.0;
    correction_ = 0.0;
  }

  
  double CalculateAverage(double new_value) {
    if (value_deque_.size() < window_size_) {
      
    } else {
      
      UpdateNeumaierSum(-value_deque_.front());
      value_deque_.pop_front();
    }
 
    UpdateNeumaierSum(new_value);
    value_deque_.push_back(new_value);

    return (sum_ + correction_) / double(window_size_);
  }

  std::deque<double> GetValueQueue() {
    return value_deque_;
  }
 private:
  int window_size_;
  double sum_, correction_;
  std::deque<double> value_deque_;

  void UpdateNeumaierSum(double value) {
    double new_sum = sum_ + value;
    if (std::abs(sum_) >= std::abs(value)) {
     
      correction_ += (sum_ - new_sum) + value;
    } else {
      correction_ += (value - new_sum) + sum_;
    }
    sum_ = new_sum;
  }
};
#endif 