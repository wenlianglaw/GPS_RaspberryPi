#pragma once

#define  RUN_TEST(FuncName) {\
  printf("============Testing %s==============\n", #FuncName);\
  FuncName();\
  printf("=======================Tests Ends========================\n\n");\
}
#define TEST(FuncName) void FuncName()


