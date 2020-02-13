#include "multiThreadTest.hpp"
#include "baseStdLibs.hpp"
#include <unistd.h>

std::mutex dataLock;
std::condition_variable dataCondition;

void printA() {
  for (unsigned int i = 0; i < 10; i++) {
    std::cout << i << ": this is function AAAAA!!!\n";
    sleep(1);
  }
}
void printB() {
  for (unsigned int i = 0; i < 10; i++) {
    std::cout << i << ": this is function BBBBB!!!\n";
    sleep(1);
  }
}

void addNum(int &num) {
  while (1) {
    std::unique_lock<std::mutex> guardLock(dataLock);
    num++;
    std::cout << "\n add num is " << num;
    if (1) {
      guardLock.unlock();
      dataCondition.notify_one();
      sleep(1);
    }
  }
}
void minuNum(int &num) {
  for (int i = 0; i < 10; i++) {
    std::unique_lock<std::mutex> guardLock(dataLock);
    dataCondition.wait(guardLock, [&num] {
      std::cout << "\n num is " << num;
      return num > 5;
    });
    num--;
    std::cout << "\n minu num is " << num;
  }
}