// Copyright 2017 The Abseil Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ceres/internal/fixed_array.h"

#include <stdio.h>

#include <cstring>
#include <list>
#include <memory>
#include <numeric>
#include <scoped_allocator>
#include <stdexcept>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::ElementsAreArray;

namespace {

// CERES_INTERNAL_ARRAYSIZE()
//
// Returns the number of elements in an array as a compile-time constant, which
// can be used in defining new arrays. If you use this macro on a pointer by
// mistake, you will get a compile-time error.
#define CERES_INTERNAL_ARRAYSIZE(array) (sizeof(ArraySizeHelper(array)))

// Note: this internal template function declaration is used by
// CERES_INTERNAL_ARRAYSIZE. The function doesn't need a definition, as we only
// use its type.
template <typename T, size_t N>
auto ArraySizeHelper(const T (&array)[N]) -> char (&)[N];

// Helper routine to determine if a ceres::internal::FixedArray used stack
// allocation.
template <typename ArrayType>
static bool IsOnStack(const ArrayType& a) {
  return a.size() <= ArrayType::inline_elements;
}

class ConstructionTester {
 public:
  ConstructionTester() : self_ptr_(this), value_(0) { constructions++; }
  ~ConstructionTester() {
    assert(self_ptr_ == this);
    self_ptr_ = nullptr;
    destructions++;
  }

  // These are incremented as elements are constructed and destructed so we can
  // be sure all elements are properly cleaned up.
  static int constructions;
  static int destructions;

  void CheckConstructed() { assert(self_ptr_ == this); }

  void set(int value) { value_ = value; }
  int get() { return value_; }

 private:
  // self_ptr_ should always point to 'this' -- that's how we can be sure the
  // constructor has been called.
  ConstructionTester* self_ptr_;
  int value_;
};

int ConstructionTester::constructions = 0;
int ConstructionTester::destructions = 0;

// ThreeInts will initialize its three ints to the value stored in
// ThreeInts::counter. The constructor increments counter so that each object
// in an array of ThreeInts will have different values.
class ThreeInts {
 public:
  ThreeInts() {
    x_ = counter;
    y_ = counter;
    z_ = counter;
    ++counter;
  }

  static int counter;

  int x_, y_, z_;
};

int ThreeInts::counter = 0;

TEST(FixedArrayTest, CopyCtor) {
  ceres::internal::FixedArray<int, 10> on_stack(5);
  std::iota(on_stack.begin(), on_stack.end(), 0);
  ceres::internal::FixedArray<int, 10> stack_copy = on_stack;
  EXPECT_THAT(stack_copy, ElementsAreArray(on_stack));
  EXPECT_TRUE(IsOnStack(stack_copy));

  ceres::internal::FixedArray<int, 10> allocated(15);
  std::iota(allocated.begin(), allocated.end(), 0);
  ceres::internal::FixedArray<int, 10> alloced_copy = allocated;
  EXPECT_THAT(alloced_copy, ElementsAreArray(allocated));
  EXPECT_FALSE(IsOnStack(alloced_copy));
}

TEST(FixedArrayTest, MoveCtor) {
  ceres::internal::FixedArray<std::unique_ptr<int>, 10> on_stack(5);
  for (int i = 0; i < 5; ++i) {
    on_stack[i] = std::unique_ptr<int>(new int(i));
  }

  ceres::internal::FixedArray<std::unique_ptr<int>, 10> stack_copy =
      std::move(on_stack);
  for (int i = 0; i < 5; ++i) EXPECT_EQ(*(stack_copy[i]), i);
  EXPECT_EQ(stack_copy.size(), on_stack.size());

  ceres::internal::FixedArray<std::unique_ptr<int>, 10> allocated(15);
  for (int i = 0; i < 15; ++i) {
    allocated[i] = std::unique_ptr<int>(new int(i));
  }

  ceres::internal::FixedArray<std::unique_ptr<int>, 10> alloced_copy =
      std::move(allocated);
  for (int i = 0; i < 15; ++i) EXPECT_EQ(*(alloced_copy[i]), i);
  EXPECT_EQ(allocated.size(), alloced_copy.size());
}

TEST(FixedArrayTest, SmallObjects) {
  // Small object arrays
  {
    // Short arrays should be on the stack
    ceres::internal::FixedArray<int> array(4);
    EXPECT_TRUE(IsOnStack(array));
  }

  {
    // Large arrays should be on the heap
    ceres::internal::FixedArray<int> array(1048576);
    EXPECT_FALSE(IsOnStack(array));
  }

  {
    // Arrays of <= default size should be on the stack
    ceres::internal::FixedArray<int, 100> array(100);
    EXPECT_TRUE(IsOnStack(array));
  }

  {
    // Arrays of > default size should be on the heap
    ceres::internal::FixedArray<int, 100> array(101);
    EXPECT_FALSE(IsOnStack(array));
  }

  {
    // Arrays with different size elements should use approximately
    // same amount of stack space
    ceres::internal::FixedArray<int> array1(0);
    ceres::internal::FixedArray<char> array2(0);
    EXPECT_LE(sizeof(array1), sizeof(array2) + 100);
    EXPECT_LE(sizeof(array2), sizeof(array1) + 100);
  }

  {
    // Ensure that vectors are properly constructed inside a fixed array.
    ceres::internal::FixedArray<std::vector<int>> array(2);
    EXPECT_EQ(0, array[0].size());
    EXPECT_EQ(0, array[1].size());
  }

  {
    // Regardless of ceres::internal::FixedArray implementation, check that a
    // type with a low alignment requirement and a non power-of-two size is
    // initialized correctly.
    ThreeInts::counter = 1;
    ceres::internal::FixedArray<ThreeInts> array(2);
    EXPECT_EQ(1, array[0].x_);
    EXPECT_EQ(1, array[0].y_);
    EXPECT_EQ(1, array[0].z_);
    EXPECT_EQ(2, array[1].x_);
    EXPECT_EQ(2, array[1].y_);
    EXPECT_EQ(2, array[1].z_);
  }
}

TEST(FixedArrayRelationalsTest, EqualArrays) {
  for (int i = 0; i < 10; ++i) {
    ceres::internal::FixedArray<int, 5> a1(i);
    std::iota(a1.begin(), a1.end(), 0);
    ceres::internal::FixedArray<int, 5> a2(a1.begin(), a1.end());

    EXPECT_TRUE(a1 == a2);
    EXPECT_FALSE(a1 != a2);
    EXPECT_TRUE(a2 == a1);
    EXPECT_FALSE(a2 != a1);
    EXPECT_FALSE(a1 < a2);
    EXPECT_FALSE(a1 > a2);
    EXPECT_FALSE(a2 < a1);
    EXPECT_FALSE(a2 > a1);
    EXPECT_TRUE(a1 <= a2);
    EXPECT_TRUE(a1 >= a2);
    EXPECT_TRUE(a2 <= a1);
    EXPECT_TRUE(a2 >= a1);
  }
}

TEST(FixedArrayRelationalsTest, UnequalArrays) {
  for (int i = 1; i < 10; ++i) {
    ceres::internal::FixedArray<int, 5> a1(i);
    std::iota(a1.begin(), a1.end(), 0);
    ceres::internal::FixedArray<int, 5> a2(a1.begin(), a1.end());
    --a2[i / 2];

    EXPECT_FALSE(a1 == a2);
    EXPECT_TRUE(a1 != a2);
    EXPECT_FALSE(a2 == a1);
    EXPECT_TRUE(a2 != a1);
    EXPECT_FALSE(a1 < a2);
    EXPECT_TRUE(a1 > a2);
    EXPECT_TRUE(a2 < a1);
    EXPECT_FALSE(a2 > a1);
    EXPECT_FALSE(a1 <= a2);
    EXPECT_TRUE(a1 >= a2);
    EXPECT_TRUE(a2 <= a1);
    EXPECT_FALSE(a2 >= a1);
  }
}

template <int stack_elements>
static void TestArray(int n) {
  SCOPED_TRACE(n);
  SCOPED_TRACE(stack_elements);
  ConstructionTester::constructions = 0;
  ConstructionTester::destructions = 0;
  {
    ceres::internal::FixedArray<ConstructionTester, stack_elements> array(n);

    EXPECT_THAT(array.size(), n);
    EXPECT_THAT(array.memsize(), sizeof(ConstructionTester) * n);
    EXPECT_THAT(array.begin() + n, array.end());

    // Check that all elements were constructed
    for (int i = 0; i < n; i++) {
      array[i].CheckConstructed();
    }
    // Check that no other elements were constructed
    EXPECT_THAT(ConstructionTester::constructions, n);

    // Test operator[]
    for (int i = 0; i < n; i++) {
      array[i].set(i);
    }
    for (int i = 0; i < n; i++) {
      EXPECT_THAT(array[i].get(), i);
      EXPECT_THAT(array.data()[i].get(), i);
    }

    // Test data()
    for (int i = 0; i < n; i++) {
      array.data()[i].set(i + 1);
    }
    for (int i = 0; i < n; i++) {
      EXPECT_THAT(array[i].get(), i + 1);
      EXPECT_THAT(array.data()[i].get(), i + 1);
    }
  }  // Close scope containing 'array'.

  // Check that all constructed elements were destructed.
  EXPECT_EQ(ConstructionTester::constructions,
            ConstructionTester::destructions);
}

template <int elements_per_inner_array, int inline_elements>
static void TestArrayOfArrays(int n) {
  SCOPED_TRACE(n);
  SCOPED_TRACE(inline_elements);
  SCOPED_TRACE(elements_per_inner_array);
  ConstructionTester::constructions = 0;
  ConstructionTester::destructions = 0;
  {
    using InnerArray = ConstructionTester[elements_per_inner_array];
    // Heap-allocate the FixedArray to avoid blowing the stack frame.
    auto array_ptr = std::unique_ptr<
        ceres::internal::FixedArray<InnerArray, inline_elements>>(
        new ceres::internal::FixedArray<InnerArray, inline_elements>(n));
    auto& array = *array_ptr;

    ASSERT_EQ(array.size(), n);
    ASSERT_EQ(array.memsize(),
              sizeof(ConstructionTester) * elements_per_inner_array * n);
    ASSERT_EQ(array.begin() + n, array.end());

    // Check that all elements were constructed
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < elements_per_inner_array; j++) {
        (array[i])[j].CheckConstructed();
      }
    }
    // Check that no other elements were constructed
    ASSERT_EQ(ConstructionTester::constructions, n * elements_per_inner_array);

    // Test operator[]
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < elements_per_inner_array; j++) {
        (array[i])[j].set(i * elements_per_inner_array + j);
      }
    }
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < elements_per_inner_array; j++) {
        ASSERT_EQ((array[i])[j].get(), i * elements_per_inner_array + j);
        ASSERT_EQ((array.data()[i])[j].get(), i * elements_per_inner_array + j);
      }
    }

    // Test data()
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < elements_per_inner_array; j++) {
        (array.data()[i])[j].set((i + 1) * elements_per_inner_array + j);
      }
    }
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < elements_per_inner_array; j++) {
        ASSERT_EQ((array[i])[j].get(), (i + 1) * elements_per_inner_array + j);
        ASSERT_EQ((array.data()[i])[j].get(),
                  (i + 1) * elements_per_inner_array + j);
      }
    }
  }  // Close scope containing 'array'.

  // Check that all constructed elements were destructed.
  EXPECT_EQ(ConstructionTester::constructions,
            ConstructionTester::destructions);
}

TEST(IteratorConstructorTest, NonInline) {
  int const kInput[] = {2, 3, 5, 7, 11, 13, 17};
  ceres::internal::FixedArray<int, CERES_INTERNAL_ARRAYSIZE(kInput) - 1> const
      fixed(kInput, kInput + CERES_INTERNAL_ARRAYSIZE(kInput));
  ASSERT_EQ(CERES_INTERNAL_ARRAYSIZE(kInput), fixed.size());
  for (size_t i = 0; i < CERES_INTERNAL_ARRAYSIZE(kInput); ++i) {
    ASSERT_EQ(kInput[i], fixed[i]);
  }
}

TEST(IteratorConstructorTest, Inline) {
  int const kInput[] = {2, 3, 5, 7, 11, 13, 17};
  ceres::internal::FixedArray<int, CERES_INTERNAL_ARRAYSIZE(kInput)> const
      fixed(kInput, kInput + CERES_INTERNAL_ARRAYSIZE(kInput));
  ASSERT_EQ(CERES_INTERNAL_ARRAYSIZE(kInput), fixed.size());
  for (size_t i = 0; i < CERES_INTERNAL_ARRAYSIZE(kInput); ++i) {
    ASSERT_EQ(kInput[i], fixed[i]);
  }
}

TEST(IteratorConstructorTest, NonPod) {
  char const* kInput[] = {
      "red", "orange", "yellow", "green", "blue", "indigo", "violet"};
  ceres::internal::FixedArray<std::string> const fixed(
      kInput, kInput + CERES_INTERNAL_ARRAYSIZE(kInput));
  ASSERT_EQ(CERES_INTERNAL_ARRAYSIZE(kInput), fixed.size());
  for (size_t i = 0; i < CERES_INTERNAL_ARRAYSIZE(kInput); ++i) {
    ASSERT_EQ(kInput[i], fixed[i]);
  }
}

TEST(IteratorConstructorTest, FromEmptyVector) {
  std::vector<int> const empty;
  ceres::internal::FixedArray<int> const fixed(empty.begin(), empty.end());
  EXPECT_EQ(0, fixed.size());
  EXPECT_EQ(empty.size(), fixed.size());
}

TEST(IteratorConstructorTest, FromNonEmptyVector) {
  int const kInput[] = {2, 3, 5, 7, 11, 13, 17};
  std::vector<int> const items(kInput,
                               kInput + CERES_INTERNAL_ARRAYSIZE(kInput));
  ceres::internal::FixedArray<int> const fixed(items.begin(), items.end());
  ASSERT_EQ(items.size(), fixed.size());
  for (size_t i = 0; i < items.size(); ++i) {
    ASSERT_EQ(items[i], fixed[i]);
  }
}

TEST(IteratorConstructorTest, FromBidirectionalIteratorRange) {
  int const kInput[] = {2, 3, 5, 7, 11, 13, 17};
  std::list<int> const items(kInput, kInput + CERES_INTERNAL_ARRAYSIZE(kInput));
  ceres::internal::FixedArray<int> const fixed(items.begin(), items.end());
  EXPECT_THAT(fixed, testing::ElementsAreArray(kInput));
}

TEST(InitListConstructorTest, InitListConstruction) {
  ceres::internal::FixedArray<int> fixed = {1, 2, 3};
  EXPECT_THAT(fixed, testing::ElementsAreArray({1, 2, 3}));
}

TEST(FillConstructorTest, NonEmptyArrays) {
  ceres::internal::FixedArray<int> stack_array(4, 1);
  EXPECT_THAT(stack_array, testing::ElementsAreArray({1, 1, 1, 1}));

  ceres::internal::FixedArray<int, 0> heap_array(4, 1);
  EXPECT_THAT(stack_array, testing::ElementsAreArray({1, 1, 1, 1}));
}

TEST(FillConstructorTest, EmptyArray) {
  ceres::internal::FixedArray<int> empty_fill(0, 1);
  ceres::internal::FixedArray<int> empty_size(0);
  EXPECT_EQ(empty_fill, empty_size);
}

TEST(FillConstructorTest, NotTriviallyCopyable) {
  std::string str = "abcd";
  ceres::internal::FixedArray<std::string> strings = {str, str, str, str};

  ceres::internal::FixedArray<std::string> array(4, str);
  EXPECT_EQ(array, strings);
}

TEST(FillConstructorTest, Disambiguation) {
  ceres::internal::FixedArray<size_t> a(1, 2);
  EXPECT_THAT(a, testing::ElementsAre(2));
}

TEST(FixedArrayTest, ManySizedArrays) {
  std::vector<int> sizes;
  for (int i = 1; i < 100; i++) sizes.push_back(i);
  for (int i = 100; i <= 1000; i += 100) sizes.push_back(i);
  for (int n : sizes) {
    TestArray<0>(n);
    TestArray<1>(n);
    TestArray<64>(n);
    TestArray<1000>(n);
  }
}

TEST(FixedArrayTest, ManySizedArraysOfArraysOf1) {
  for (int n = 1; n < 1000; n++) {
    ASSERT_NO_FATAL_FAILURE((TestArrayOfArrays<1, 0>(n)));
    ASSERT_NO_FATAL_FAILURE((TestArrayOfArrays<1, 1>(n)));
    ASSERT_NO_FATAL_FAILURE((TestArrayOfArrays<1, 64>(n)));
    ASSERT_NO_FATAL_FAILURE((TestArrayOfArrays<1, 1000>(n)));
  }
}

TEST(FixedArrayTest, ManySizedArraysOfArraysOf2) {
  for (int n = 1; n < 1000; n++) {
    TestArrayOfArrays<2, 0>(n);
    TestArrayOfArrays<2, 1>(n);
    TestArrayOfArrays<2, 64>(n);
    TestArrayOfArrays<2, 1000>(n);
  }
}

// If value_type is put inside of a struct container,
// we might evoke this error in a hardened build unless data() is carefully
// written, so check on that.
//     error: call to int __builtin___sprintf_chk(etc...)
//     will always overflow destination buffer [-Werror]
TEST(FixedArrayTest, AvoidParanoidDiagnostics) {
  ceres::internal::FixedArray<char, 32> buf(32);
  sprintf(buf.data(), "foo");  // NOLINT(runtime/printf)
}

TEST(FixedArrayTest, TooBigInlinedSpace) {
  struct TooBig {
    char c[1 << 20];
  };  // too big for even one on the stack

  // Simulate the data members of ceres::internal::FixedArray, a pointer and a
  // size_t.
  struct Data {
    std::tuple<size_t, std::allocator<double>> size_alloc_;
    TooBig* p;
  };

  // Make sure TooBig objects are not inlined for 0 or default size.
  static_assert(
      sizeof(ceres::internal::FixedArray<TooBig, 0>) == sizeof(Data),
      "0-sized ceres::internal::FixedArray should have same size as Data.");
  static_assert(
      alignof(ceres::internal::FixedArray<TooBig, 0>) == alignof(Data),
      "0-sized ceres::internal::FixedArray should have same alignment as "
      "Data.");
  static_assert(sizeof(ceres::internal::FixedArray<TooBig>) == sizeof(Data),
                "default-sized ceres::internal::FixedArray should have same "
                "size as Data");
  static_assert(alignof(ceres::internal::FixedArray<TooBig>) == alignof(Data),
                "default-sized ceres::internal::FixedArray should have same "
                "alignment as Data.");
}

// PickyDelete EXPECTs its class-scope deallocation funcs are unused.
struct PickyDelete {
  PickyDelete() {}
  ~PickyDelete() {}
  void operator delete(void* p) {
    EXPECT_TRUE(false) << __FUNCTION__;
    ::operator delete(p);
  }
  void operator delete[](void* p) {
    EXPECT_TRUE(false) << __FUNCTION__;
    ::operator delete[](p);
  }
};

TEST(FixedArrayTest, UsesGlobalAlloc) {
  ceres::internal::FixedArray<PickyDelete, 0> a(5);
}

TEST(FixedArrayTest, Data) {
  static const int kInput[] = {2, 3, 5, 7, 11, 13, 17};
  ceres::internal::FixedArray<int> fa(std::begin(kInput), std::end(kInput));
  EXPECT_EQ(fa.data(), &*fa.begin());
  EXPECT_EQ(fa.data(), &fa[0]);

  const ceres::internal::FixedArray<int>& cfa = fa;
  EXPECT_EQ(cfa.data(), &*cfa.begin());
  EXPECT_EQ(cfa.data(), &cfa[0]);
}

TEST(FixedArrayTest, Empty) {
  ceres::internal::FixedArray<int> empty(0);
  ceres::internal::FixedArray<int> inline_filled(1);
  ceres::internal::FixedArray<int, 0> heap_filled(1);
  EXPECT_TRUE(empty.empty());
  EXPECT_FALSE(inline_filled.empty());
  EXPECT_FALSE(heap_filled.empty());
}

TEST(FixedArrayTest, FrontAndBack) {
  ceres::internal::FixedArray<int, 3 * sizeof(int)> inlined = {1, 2, 3};
  EXPECT_EQ(inlined.front(), 1);
  EXPECT_EQ(inlined.back(), 3);

  ceres::internal::FixedArray<int, 0> allocated = {1, 2, 3};
  EXPECT_EQ(allocated.front(), 1);
  EXPECT_EQ(allocated.back(), 3);

  ceres::internal::FixedArray<int> one_element = {1};
  EXPECT_EQ(one_element.front(), one_element.back());
}

TEST(FixedArrayTest, ReverseIteratorInlined) {
  ceres::internal::FixedArray<int, 5 * sizeof(int)> a = {0, 1, 2, 3, 4};

  int counter = 5;
  for (ceres::internal::FixedArray<int>::reverse_iterator iter = a.rbegin();
       iter != a.rend();
       ++iter) {
    counter--;
    EXPECT_EQ(counter, *iter);
  }
  EXPECT_EQ(counter, 0);

  counter = 5;
  for (ceres::internal::FixedArray<int>::const_reverse_iterator iter =
           a.rbegin();
       iter != a.rend();
       ++iter) {
    counter--;
    EXPECT_EQ(counter, *iter);
  }
  EXPECT_EQ(counter, 0);

  counter = 5;
  for (auto iter = a.crbegin(); iter != a.crend(); ++iter) {
    counter--;
    EXPECT_EQ(counter, *iter);
  }
  EXPECT_EQ(counter, 0);
}

TEST(FixedArrayTest, ReverseIteratorAllocated) {
  ceres::internal::FixedArray<int, 0> a = {0, 1, 2, 3, 4};

  int counter = 5;
  for (ceres::internal::FixedArray<int>::reverse_iterator iter = a.rbegin();
       iter != a.rend();
       ++iter) {
    counter--;
    EXPECT_EQ(counter, *iter);
  }
  EXPECT_EQ(counter, 0);

  counter = 5;
  for (ceres::internal::FixedArray<int>::const_reverse_iterator iter =
           a.rbegin();
       iter != a.rend();
       ++iter) {
    counter--;
    EXPECT_EQ(counter, *iter);
  }
  EXPECT_EQ(counter, 0);

  counter = 5;
  for (auto iter = a.crbegin(); iter != a.crend(); ++iter) {
    counter--;
    EXPECT_EQ(counter, *iter);
  }
  EXPECT_EQ(counter, 0);
}

TEST(FixedArrayTest, Fill) {
  ceres::internal::FixedArray<int, 5 * sizeof(int)> inlined(5);
  int fill_val = 42;
  inlined.fill(fill_val);
  for (int i : inlined) EXPECT_EQ(i, fill_val);

  ceres::internal::FixedArray<int, 0> allocated(5);
  allocated.fill(fill_val);
  for (int i : allocated) EXPECT_EQ(i, fill_val);

  // It doesn't do anything, just make sure this compiles.
  ceres::internal::FixedArray<int> empty(0);
  empty.fill(fill_val);
}

// TODO(johnsoncj): Investigate InlinedStorage default initialization in GCC 4.x
#ifndef __GNUC__
TEST(FixedArrayTest, DefaultCtorDoesNotValueInit) {
  using T = char;
  constexpr auto capacity = 10;
  using FixedArrType = ceres::internal::FixedArray<T, capacity>;
  using FixedArrBuffType =
      typename std::aligned_storage<sizeof(FixedArrType),
                                    alignof(FixedArrType)>::type;
  constexpr auto scrubbed_bits = 0x95;
  constexpr auto length = capacity / 2;

  FixedArrBuffType buff;
  std::memset(std::addressof(buff), scrubbed_bits, sizeof(FixedArrBuffType));

  FixedArrType* arr =
      ::new (static_cast<void*>(std::addressof(buff))) FixedArrType(length);
  EXPECT_THAT(*arr, testing::Each(scrubbed_bits));
  arr->~FixedArrType();
}
#endif  // __GNUC__

// This is a stateful allocator, but the state lives outside of the
// allocator (in whatever test is using the allocator). This is odd
// but helps in tests where the allocator is propagated into nested
// containers - that chain of allocators uses the same state and is
// thus easier to query for aggregate allocation information.
template <typename T>
class CountingAllocator : public std::allocator<T> {
 public:
  using Alloc = std::allocator<T>;
  using pointer = typename Alloc::pointer;
  using size_type = typename Alloc::size_type;

  CountingAllocator() : bytes_used_(nullptr), instance_count_(nullptr) {}
  explicit CountingAllocator(int64_t* b)
      : bytes_used_(b), instance_count_(nullptr) {}
  CountingAllocator(int64_t* b, int64_t* a)
      : bytes_used_(b), instance_count_(a) {}

  template <typename U>
  explicit CountingAllocator(const CountingAllocator<U>& x)
      : Alloc(x),
        bytes_used_(x.bytes_used_),
        instance_count_(x.instance_count_) {}

  pointer allocate(size_type n, const void* const hint = nullptr) {
    assert(bytes_used_ != nullptr);
    *bytes_used_ += n * sizeof(T);
    return Alloc::allocate(n, hint);
  }

  void deallocate(pointer p, size_type n) {
    Alloc::deallocate(p, n);
    assert(bytes_used_ != nullptr);
    *bytes_used_ -= n * sizeof(T);
  }

  template <typename... Args>
  void construct(pointer p, Args&&... args) {
    Alloc::construct(p, std::forward<Args>(args)...);
    if (instance_count_) {
      *instance_count_ += 1;
    }
  }

  void destroy(pointer p) {
    Alloc::destroy(p);
    if (instance_count_) {
      *instance_count_ -= 1;
    }
  }

  template <typename U>
  class rebind {
   public:
    using other = CountingAllocator<U>;
  };

  int64_t* bytes_used_;
  int64_t* instance_count_;
};

TEST(AllocatorSupportTest, CountInlineAllocations) {
  constexpr size_t inlined_size = 4;
  using Alloc = CountingAllocator<int>;
  using AllocFxdArr = ceres::internal::FixedArray<int, inlined_size, Alloc>;

  int64_t allocated = 0;
  int64_t active_instances = 0;

  {
    const int ia[] = {0, 1, 2, 3, 4, 5, 6, 7};

    Alloc alloc(&allocated, &active_instances);

    AllocFxdArr arr(ia, ia + inlined_size, alloc);
    static_cast<void>(arr);
  }

  EXPECT_EQ(allocated, 0);
  EXPECT_EQ(active_instances, 0);
}

TEST(AllocatorSupportTest, CountOutoflineAllocations) {
  constexpr size_t inlined_size = 4;
  using Alloc = CountingAllocator<int>;
  using AllocFxdArr = ceres::internal::FixedArray<int, inlined_size, Alloc>;

  int64_t allocated = 0;
  int64_t active_instances = 0;

  {
    const int ia[] = {0, 1, 2, 3, 4, 5, 6, 7};
    Alloc alloc(&allocated, &active_instances);

    AllocFxdArr arr(ia, ia + CERES_INTERNAL_ARRAYSIZE(ia), alloc);

    EXPECT_EQ(allocated, arr.size() * sizeof(int));
    static_cast<void>(arr);
  }

  EXPECT_EQ(active_instances, 0);
}

TEST(AllocatorSupportTest, CountCopyInlineAllocations) {
  constexpr size_t inlined_size = 4;
  using Alloc = CountingAllocator<int>;
  using AllocFxdArr = ceres::internal::FixedArray<int, inlined_size, Alloc>;

  int64_t allocated1 = 0;
  int64_t allocated2 = 0;
  int64_t active_instances = 0;
  Alloc alloc(&allocated1, &active_instances);
  Alloc alloc2(&allocated2, &active_instances);

  {
    int initial_value = 1;

    AllocFxdArr arr1(inlined_size / 2, initial_value, alloc);

    EXPECT_EQ(allocated1, 0);

    AllocFxdArr arr2(arr1, alloc2);

    EXPECT_EQ(allocated2, 0);
    static_cast<void>(arr1);
    static_cast<void>(arr2);
  }

  EXPECT_EQ(active_instances, 0);
}

TEST(AllocatorSupportTest, CountCopyOutoflineAllocations) {
  constexpr size_t inlined_size = 4;
  using Alloc = CountingAllocator<int>;
  using AllocFxdArr = ceres::internal::FixedArray<int, inlined_size, Alloc>;

  int64_t allocated1 = 0;
  int64_t allocated2 = 0;
  int64_t active_instances = 0;
  Alloc alloc(&allocated1, &active_instances);
  Alloc alloc2(&allocated2, &active_instances);

  {
    int initial_value = 1;

    AllocFxdArr arr1(inlined_size * 2, initial_value, alloc);

    EXPECT_EQ(allocated1, arr1.size() * sizeof(int));

    AllocFxdArr arr2(arr1, alloc2);

    EXPECT_EQ(allocated2, inlined_size * 2 * sizeof(int));
    static_cast<void>(arr1);
    static_cast<void>(arr2);
  }

  EXPECT_EQ(active_instances, 0);
}

TEST(AllocatorSupportTest, SizeValAllocConstructor) {
  using testing::AllOf;
  using testing::Each;
  using testing::SizeIs;

  constexpr size_t inlined_size = 4;
  using Alloc = CountingAllocator<int>;
  using AllocFxdArr = ceres::internal::FixedArray<int, inlined_size, Alloc>;

  {
    auto len = inlined_size / 2;
    auto val = 0;
    int64_t allocated = 0;
    AllocFxdArr arr(len, val, Alloc(&allocated));

    EXPECT_EQ(allocated, 0);
    EXPECT_THAT(arr, AllOf(SizeIs(len), Each(0)));
  }

  {
    auto len = inlined_size * 2;
    auto val = 0;
    int64_t allocated = 0;
    AllocFxdArr arr(len, val, Alloc(&allocated));

    EXPECT_EQ(allocated, len * sizeof(int));
    EXPECT_THAT(arr, AllOf(SizeIs(len), Each(0)));
  }
}

struct EigenStruct {
  Eigen::Vector4d data;
};

static_assert(
    std::is_same<ceres::internal::FixedArrayDefaultAllocator<double>,
                 std::allocator<double>>::value,
    "Double is a trivial type, so std::allocator should be used here.");
static_assert(
    std::is_same<ceres::internal::FixedArrayDefaultAllocator<double*>,
                 std::allocator<double*>>::value,
    "A pointer is a trivial type, so std::allocator should be used here.");
static_assert(
    std::is_same<ceres::internal::FixedArrayDefaultAllocator<Eigen::Matrix4d>,
                 Eigen::aligned_allocator<Eigen::Matrix4d>>::value,
    "An Eigen::Matrix4d needs the Eigen::aligned_allocator for proper "
    "alignment.");
static_assert(
    std::is_same<ceres::internal::FixedArrayDefaultAllocator<EigenStruct>,
                 Eigen::aligned_allocator<EigenStruct>>::value,
    "A struct containing fixed size Eigen types needs Eigen::aligned_allocator "
    "for proper alignment.");

}  // namespace
