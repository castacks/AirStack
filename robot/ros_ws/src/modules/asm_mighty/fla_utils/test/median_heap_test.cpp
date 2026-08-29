// median_heap_test
#include <gtest/gtest.h>

#include "../src/median_heap.h"

using fla_utils::MedianHeap;

// Basic test
TEST(MedianHeap, Basic)
{
  MedianHeap<int> mh;

  ASSERT_TRUE( mh.empty() );
  mh.pop(-100);
  ASSERT_TRUE( mh.empty() );
  ASSERT_EQ( mh.size(), 0 );

  mh.push(0);
  ASSERT_FALSE( mh.empty() );
  ASSERT_EQ( mh.size(), 1 );
  ASSERT_EQ( mh.get_median(), 0 );

  ASSERT_FALSE( mh.pop(1) );
  ASSERT_FALSE( mh.empty() );
  ASSERT_EQ( mh.size(), 1 );
  ASSERT_EQ( mh.get_median(), 0 );

  ASSERT_TRUE( mh.pop(0) );
  ASSERT_TRUE( mh.empty() );
  ASSERT_EQ( mh.size(), 0 );
}

TEST(MedianHeap, Multiple)
{
  MedianHeap<int> mh;

  ASSERT_TRUE( mh.empty() );
  mh.push(2);
  mh.push(5);
  mh.push(7);
  mh.push(2);
  mh.push(3);
  mh.push(9);
  mh.push(3);
  mh.push(0);
  mh.push(1);

  ASSERT_FALSE( mh.empty() );
  ASSERT_EQ( mh.size(), 9 );
  ASSERT_EQ( mh.get_median(), 3 );
  ASSERT_TRUE( mh.pop(2) );
  ASSERT_TRUE( mh.pop(2) );
  ASSERT_TRUE( mh.pop(3) );

  ASSERT_FALSE( mh.empty() );
  ASSERT_EQ( mh.size(), 6 );
  ASSERT_EQ( mh.get_median(), 4 );

  mh.push(9);
  mh.push(13);
  mh.push(17);

  ASSERT_FALSE( mh.empty() );
  ASSERT_EQ( mh.size(), 9 );
  ASSERT_EQ( mh.get_median(), 7 );

  mh.push(1511);
  mh.push(-20);
  mh.push(1513);

  ASSERT_FALSE( mh.empty() );
  ASSERT_EQ( mh.size(), 12 );
  ASSERT_EQ( mh.get_median(), 8 );

  ASSERT_TRUE( mh.pop(3) );
  ASSERT_TRUE( mh.pop(5) );
  ASSERT_FALSE( mh.pop(5) );

  ASSERT_FALSE( mh.empty() );
  ASSERT_EQ( mh.size(), 10 );
  ASSERT_EQ( mh.get_median(), 9 );

  mh.clear();
  ASSERT_TRUE( mh.empty() );
  ASSERT_EQ( mh.size(), 0 );
}

TEST(MedianHeap, Multiple2)
{
  MedianHeap<int> mh;

  ASSERT_TRUE( mh.empty() );
  mh.push(5316);
  mh.push(5316);
  mh.push(5316);
  mh.push(5316);

  ASSERT_EQ( mh.get_median(), 5316 );
  ASSERT_EQ( mh.size(), 4 );

  mh.push(5316);

  ASSERT_TRUE( mh.pop(5316) );
  ASSERT_TRUE( mh.pop(5316) );

  ASSERT_EQ( mh.get_median(), 5316 );
  ASSERT_EQ( mh.size(), 3 );

  ASSERT_TRUE( mh.pop(5316) );

  mh.push(5357);

  ASSERT_TRUE( mh.pop(5316) );

  ASSERT_EQ( mh.get_median(), 5336 );

  ASSERT_TRUE( mh.pop(5316) );
  
  ASSERT_FALSE( mh.empty() );
  ASSERT_EQ( mh.size(), 1 );
  ASSERT_EQ( mh.get_median(), 5357 );
}

int main(int argc, char** argv)
{
  srand(0);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
