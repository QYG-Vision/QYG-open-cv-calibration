#include <gtest/gtest.h>
#include "thread_safe_queue.hpp"

TEST(ThreadSafeQueue, BasicPushPop) {
    tools::ThreadSafeQueue<int> q(5);
    q.push(1);
    q.push(2);
    q.push(3);

    EXPECT_EQ(q.pop(), 1);
    EXPECT_EQ(q.pop(), 2);
    EXPECT_EQ(q.pop(), 3);
}

TEST(ThreadSafeQueue, FrontAndBack) {
    tools::ThreadSafeQueue<int> q(5);
    q.push(10);
    q.push(20);

    EXPECT_EQ(q.front(), 10);

    int back_val = 0;
    q.back(back_val);
    EXPECT_EQ(back_val, 20);
}

TEST(ThreadSafeQueue, EmptyCheck) {
    tools::ThreadSafeQueue<int> q(5);
    EXPECT_TRUE(q.empty());

    q.push(1);
    EXPECT_FALSE(q.empty());

    q.pop();
    EXPECT_TRUE(q.empty());
}

TEST(ThreadSafeQueue, FullHandlerCalled) {
    int full_count = 0;
    tools::ThreadSafeQueue<int> q(
        2,
        [&full_count]() { full_count++; }
    );

    q.push(1);
    q.push(2);
    EXPECT_EQ(full_count, 0);

    q.push(3);  // queue is full, handler called
    EXPECT_EQ(full_count, 1);

    q.push(4);  // still full, handler called again
    EXPECT_EQ(full_count, 2);

    // queue still has only first two elements
    EXPECT_EQ(q.pop(), 1);
    EXPECT_EQ(q.pop(), 2);
    EXPECT_TRUE(q.empty());
}

TEST(ThreadSafeQueue, PopWhenFullOverwritesOldest) {
    tools::ThreadSafeQueue<int, true> q(3);

    q.push(1);
    q.push(2);
    q.push(3);
    q.push(4);  // should pop 1, then push 4

    EXPECT_EQ(q.pop(), 2);
    EXPECT_EQ(q.pop(), 3);
    EXPECT_EQ(q.pop(), 4);
    EXPECT_TRUE(q.empty());
}

TEST(ThreadSafeQueue, PopWhenFullKeepsMaxSize) {
    tools::ThreadSafeQueue<int, true> q(2);

    q.push(10);
    q.push(20);
    q.push(30);  // 10 dropped, 20/30 remain
    q.push(40);  // 20 dropped, 30/40 remain

    EXPECT_EQ(q.pop(), 30);
    EXPECT_EQ(q.pop(), 40);
    EXPECT_TRUE(q.empty());
}

TEST(ThreadSafeQueue, ClearEmptiesQueue) {
    tools::ThreadSafeQueue<int> q(10);
    q.push(1);
    q.push(2);
    q.push(3);

    q.clear();
    EXPECT_TRUE(q.empty());
}
