#pragma once

#define  RUN_TEST(FuncName) {\
  printf("Testing %s\n", #FuncName);\
  FuncName();\
  printf("%s passed!\n\n", #FuncName);\
}
#define TEST(FuncName) void FuncName()


